#include "rtt_debug_service.hpp"

#include "../../APP/Variable.hpp"
#include "../../Task/PowerTask.hpp"
#include "logger.hpp"
#include "FreeRTOS.h"
#include "task.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace HAL::LOGGER::RTTDebug
{
namespace
{
constexpr size_t kMaxParams = 32;
constexpr size_t kCommandBufferSize = 128;
constexpr size_t kReadBufferSize = 32;

enum class StorageKind : uint8_t
{
    Float,
    Double,
    Int32,
    UInt32,
    Bool,
    Custom
};

using CustomFormatter = int (*)(char *buffer, size_t size);
using CustomSetter = bool (*)(const char *value_text, char *error_text, size_t error_size);

struct ParamSlot
{
    ParamEntry meta{};
    StorageKind storage = StorageKind::Float;
    void *value_ptr = nullptr;
    CustomFormatter formatter = nullptr;
    CustomSetter setter = nullptr;
};

ParamSlot g_slots[kMaxParams];
size_t g_slot_count = 0;
bool g_initialized = false;
char g_command_buffer[kCommandBufferSize];
size_t g_command_length = 0;

Logger &log()
{
    return Logger::getInstance();
}

void write_line(const char *fmt, ...)
{
    char buffer[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    log().printf("%s\r\n", buffer);
}

bool string_equals(const char *lhs, const char *rhs)
{
    return std::strcmp(lhs, rhs) == 0;
}

bool string_starts_with(const char *text, const char *prefix)
{
    if (prefix == nullptr || *prefix == '\0')
    {
        return true;
    }

    while (*prefix != '\0')
    {
        if (*text++ != *prefix++)
        {
            return false;
        }
    }
    return true;
}

char *skip_spaces(char *text)
{
    while (*text == ' ' || *text == '\t')
    {
        ++text;
    }
    return text;
}

char *next_token(char *&cursor)
{
    cursor = skip_spaces(cursor);
    if (*cursor == '\0')
    {
        return nullptr;
    }

    char *token = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t')
    {
        ++cursor;
    }

    if (*cursor != '\0')
    {
        *cursor++ = '\0';
    }

    return token;
}

const char *type_name(ParamType type)
{
    switch (type)
    {
    case ParamType::Float:
        return "float";
    case ParamType::Int32:
        return "int32";
    case ParamType::UInt32:
        return "uint32";
    case ParamType::Bool:
        return "bool";
    default:
        return "unknown";
    }
}

ParamSlot *find_slot_mutable(const char *name)
{
    for (size_t i = 0; i < g_slot_count; ++i)
    {
        if (string_equals(g_slots[i].meta.name, name))
        {
            return &g_slots[i];
        }
    }
    return nullptr;
}

const ParamSlot *find_slot(const char *name)
{
    for (size_t i = 0; i < g_slot_count; ++i)
    {
        if (string_equals(g_slots[i].meta.name, name))
        {
            return &g_slots[i];
        }
    }
    return nullptr;
}

bool parse_bool_value(const char *text, bool &value)
{
    if (string_equals(text, "1") || string_equals(text, "true") || string_equals(text, "on"))
    {
        value = true;
        return true;
    }
    if (string_equals(text, "0") || string_equals(text, "false") || string_equals(text, "off"))
    {
        value = false;
        return true;
    }
    return false;
}

int format_direct_value(const ParamSlot &slot, char *buffer, size_t size)
{
    int written = 0;
    taskENTER_CRITICAL();
    switch (slot.storage)
    {
    case StorageKind::Float:
        written = snprintf(buffer, size, "%.6f", static_cast<double>(*static_cast<float *>(slot.value_ptr)));
        break;
    case StorageKind::Double:
        written = snprintf(buffer, size, "%.6f", *static_cast<double *>(slot.value_ptr));
        break;
    case StorageKind::Int32:
        written = snprintf(buffer, size, "%ld", static_cast<long>(*static_cast<int32_t *>(slot.value_ptr)));
        break;
    case StorageKind::UInt32:
        written = snprintf(buffer, size, "%lu", static_cast<unsigned long>(*static_cast<uint32_t *>(slot.value_ptr)));
        break;
    case StorageKind::Bool:
        written = snprintf(buffer, size, "%s", (*static_cast<bool *>(slot.value_ptr)) ? "true" : "false");
        break;
    case StorageKind::Custom:
    default:
        written = 0;
        break;
    }
    taskEXIT_CRITICAL();
    return written;
}

bool set_direct_value(ParamSlot &slot, const char *value_text, char *error_text, size_t error_size)
{
    char *end_ptr = nullptr;
    switch (slot.storage)
    {
    case StorageKind::Float:
    {
        const float value = strtof(value_text, &end_ptr);
        if (end_ptr == value_text || *end_ptr != '\0')
        {
            snprintf(error_text, error_size, "invalid float");
            return false;
        }
        if (value < slot.meta.min_value || value > slot.meta.max_value)
        {
            snprintf(error_text, error_size, "out of range [%.3f, %.3f]", slot.meta.min_value, slot.meta.max_value);
            return false;
        }
        taskENTER_CRITICAL();
        *static_cast<float *>(slot.value_ptr) = value;
        taskEXIT_CRITICAL();
        return true;
    }
    case StorageKind::Double:
    {
        const float value = strtof(value_text, &end_ptr);
        if (end_ptr == value_text || *end_ptr != '\0')
        {
            snprintf(error_text, error_size, "invalid float");
            return false;
        }
        if (value < slot.meta.min_value || value > slot.meta.max_value)
        {
            snprintf(error_text, error_size, "out of range [%.3f, %.3f]", slot.meta.min_value, slot.meta.max_value);
            return false;
        }
        taskENTER_CRITICAL();
        *static_cast<double *>(slot.value_ptr) = static_cast<double>(value);
        taskEXIT_CRITICAL();
        return true;
    }
    case StorageKind::Int32:
    {
        const long value = strtol(value_text, &end_ptr, 0);
        if (end_ptr == value_text || *end_ptr != '\0')
        {
            snprintf(error_text, error_size, "invalid int32");
            return false;
        }
        if (value < static_cast<long>(slot.meta.min_value) || value > static_cast<long>(slot.meta.max_value))
        {
            snprintf(error_text, error_size, "out of range [%.0f, %.0f]", slot.meta.min_value, slot.meta.max_value);
            return false;
        }
        taskENTER_CRITICAL();
        *static_cast<int32_t *>(slot.value_ptr) = static_cast<int32_t>(value);
        taskEXIT_CRITICAL();
        return true;
    }
    case StorageKind::UInt32:
    {
        const unsigned long value = strtoul(value_text, &end_ptr, 0);
        if (end_ptr == value_text || *end_ptr != '\0')
        {
            snprintf(error_text, error_size, "invalid uint32");
            return false;
        }
        if (value < static_cast<unsigned long>(slot.meta.min_value) ||
            value > static_cast<unsigned long>(slot.meta.max_value))
        {
            snprintf(error_text, error_size, "out of range [%.0f, %.0f]", slot.meta.min_value, slot.meta.max_value);
            return false;
        }
        taskENTER_CRITICAL();
        *static_cast<uint32_t *>(slot.value_ptr) = static_cast<uint32_t>(value);
        taskEXIT_CRITICAL();
        return true;
    }
    case StorageKind::Bool:
    {
        bool value = false;
        if (!parse_bool_value(value_text, value))
        {
            snprintf(error_text, error_size, "invalid bool");
            return false;
        }
        taskENTER_CRITICAL();
        *static_cast<bool *>(slot.value_ptr) = value;
        taskEXIT_CRITICAL();
        return true;
    }
    case StorageKind::Custom:
    default:
        snprintf(error_text, error_size, "read-only");
        return false;
    }
}

void format_value(const ParamSlot &slot, char *buffer, size_t size)
{
    if (slot.storage == StorageKind::Custom && slot.formatter != nullptr)
    {
        slot.formatter(buffer, size);
        return;
    }

    format_direct_value(slot, buffer, size);
}

bool set_value(ParamSlot &slot, const char *value_text, char *error_text, size_t error_size)
{
    if (slot.storage == StorageKind::Custom && slot.setter != nullptr)
    {
        return slot.setter(value_text, error_text, error_size);
    }

    return set_direct_value(slot, value_text, error_text, error_size);
}

int format_wheel_full_energy(char *buffer, size_t size)
{
    taskENTER_CRITICAL();
    const float value = PowerControl.Wheel_PowerData.full_energy_j;
    taskEXIT_CRITICAL();
    return snprintf(buffer, size, "%.6f", static_cast<double>(value));
}

bool set_wheel_full_energy(const char *value_text, char *error_text, size_t error_size)
{
    char *end_ptr = nullptr;
    const float value = strtof(value_text, &end_ptr);
    if (end_ptr == value_text || *end_ptr != '\0')
    {
        snprintf(error_text, error_size, "invalid float");
        return false;
    }
    if (value < 10.0f || value > 5000.0f)
    {
        snprintf(error_text, error_size, "out of range [10.000, 5000.000]");
        return false;
    }

    taskENTER_CRITICAL();
    PowerControl.Wheel_PowerData.SetEnergyCapacity(value, PowerControl.Wheel_PowerData.abundance_ratio);
    taskEXIT_CRITICAL();
    return true;
}

int format_wheel_abundance_ratio(char *buffer, size_t size)
{
    taskENTER_CRITICAL();
    const float value = PowerControl.Wheel_PowerData.abundance_ratio;
    taskEXIT_CRITICAL();
    return snprintf(buffer, size, "%.6f", static_cast<double>(value));
}

bool set_wheel_abundance_ratio(const char *value_text, char *error_text, size_t error_size)
{
    char *end_ptr = nullptr;
    const float value = strtof(value_text, &end_ptr);
    if (end_ptr == value_text || *end_ptr != '\0')
    {
        snprintf(error_text, error_size, "invalid float");
        return false;
    }
    if (value < 0.1f || value > 0.99f)
    {
        snprintf(error_text, error_size, "out of range [0.100, 0.990]");
        return false;
    }

    taskENTER_CRITICAL();
    PowerControl.Wheel_PowerData.SetEnergyCapacity(PowerControl.Wheel_PowerData.full_energy_j, value);
    taskEXIT_CRITICAL();
    return true;
}

void register_custom_param(const char *name,
                           ParamType type,
                           float min_value,
                           float max_value,
                           const char *description,
                           CustomFormatter formatter,
                           CustomSetter setter)
{
    if (g_slot_count >= kMaxParams)
    {
        return;
    }

    ParamSlot &slot = g_slots[g_slot_count++];
    slot.meta.name = name;
    slot.meta.type = type;
    slot.meta.description = description;
    slot.meta.min_value = min_value;
    slot.meta.max_value = max_value;
    slot.storage = StorageKind::Custom;
    slot.value_ptr = nullptr;
    slot.formatter = formatter;
    slot.setter = setter;
}

void register_default_params(ParamRegistry &registry)
{
    registry.register_param("pid.wheel.kp", &Kpid_3508_vel.kp, 0.0f, 20000.0f, "3508 velocity loop kp");
    registry.register_param("pid.wheel.ki", &Kpid_3508_vel.ki, 0.0f, 10000.0f, "3508 velocity loop ki");
    registry.register_param("pid.wheel.kd", &Kpid_3508_vel.kd, 0.0f, 10000.0f, "3508 velocity loop kd");

    registry.register_param("pid.steer.kp", &Kpid_4005_vel.kp, 0.0f, 20000.0f, "4005 velocity loop kp");
    registry.register_param("pid.steer.ki", &Kpid_4005_vel.ki, 0.0f, 10000.0f, "4005 velocity loop ki");
    registry.register_param("pid.steer.kd", &Kpid_4005_vel.kd, 0.0f, 10000.0f, "4005 velocity loop kd");

    registry.register_param("pid.follow.kp", &Kpid_vw.kp, -20000.0f, 20000.0f, "yaw follow loop kp");
    registry.register_param("pid.follow.ki", &Kpid_vw.ki, -10000.0f, 10000.0f, "yaw follow loop ki");
    registry.register_param("pid.follow.kd", &Kpid_vw.kd, -10000.0f, 10000.0f, "yaw follow loop kd");

    registry.register_param("power.wheel.k1", &PowerControl.Wheel_PowerData.k1, 0.0f, 100.0f, "wheel power model k1");
    registry.register_param("power.wheel.k2", &PowerControl.Wheel_PowerData.k2, 0.0f, 100.0f, "wheel power model k2");
    registry.register_param("power.wheel.k3", &PowerControl.Wheel_PowerData.k3, 0.0f, 500.0f, "wheel power model k3");

    registry.register_param("power.steer.k1", &PowerControl.String_PowerData.k1, 0.0f, 100.0f, "steer power model k1");
    registry.register_param("power.steer.k2", &PowerControl.String_PowerData.k2, 0.0f, 100.0f, "steer power model k2");
    registry.register_param("power.steer.k3", &PowerControl.String_PowerData.k3, 0.0f, 500.0f, "steer power model k3");

    registry.register_param("power.wheel.abundance_kp",
                            &PowerControl.Wheel_PowerData.abundance_pid_param.kp,
                            0.0f,
                            1000.0f,
                            "energy abundance loop kp");
    registry.register_param("power.wheel.abundance_kd",
                            &PowerControl.Wheel_PowerData.abundance_pid_param.kd,
                            0.0f,
                            1000.0f,
                            "energy abundance loop kd");
    registry.register_param("power.wheel.poverty_kp",
                            &PowerControl.Wheel_PowerData.poverty_pid_param.kp,
                            0.0f,
                            1000.0f,
                            "energy poverty loop kp");
    registry.register_param("power.wheel.poverty_kd",
                            &PowerControl.Wheel_PowerData.poverty_pid_param.kd,
                            0.0f,
                            1000.0f,
                            "energy poverty loop kd");
    registry.register_param("power.wheel.poverty_line",
                            &PowerControl.Wheel_PowerData.poverty_line,
                            0.0f,
                            2000.0f,
                            "energy poverty threshold");
    registry.register_param("power.wheel.buffer_poverty_line",
                            &PowerControl.Wheel_PowerData.buffer_poverty_line,
                            0.0f,
                            60.0f,
                            "buffer poverty threshold");
    registry.register_param("power.wheel.charge_ratio",
                            &PowerControl.Wheel_PowerData.charge_ratio,
                            0.1f,
                            1.0f,
                            "forced charge power ratio");
    registry.register_param("power.wheel.min_power_ratio",
                            &PowerControl.Wheel_PowerData.min_power_ratio,
                            0.1f,
                            1.5f,
                            "minimum power ratio");

    register_custom_param("power.wheel.full_energy_j",
                          ParamType::Float,
                          10.0f,
                          5000.0f,
                          "virtual full energy capacity",
                          format_wheel_full_energy,
                          set_wheel_full_energy);
    register_custom_param("power.wheel.abundance_ratio",
                          ParamType::Float,
                          0.1f,
                          0.99f,
                          "abundance threshold ratio",
                          format_wheel_abundance_ratio,
                          set_wheel_abundance_ratio);
}
} // namespace

bool ParamRegistry::register_param(const char *name,
                                   float *value,
                                   float min_value,
                                   float max_value,
                                   const char *description)
{
    if (g_slot_count >= kMaxParams || value == nullptr || find_slot(name) != nullptr)
    {
        return false;
    }

    ParamSlot &slot = g_slots[g_slot_count++];
    slot.meta = {name, ParamType::Float, description, min_value, max_value};
    slot.storage = StorageKind::Float;
    slot.value_ptr = value;
    return true;
}

bool ParamRegistry::register_param(const char *name,
                                   double *value,
                                   float min_value,
                                   float max_value,
                                   const char *description)
{
    if (g_slot_count >= kMaxParams || value == nullptr || find_slot(name) != nullptr)
    {
        return false;
    }

    ParamSlot &slot = g_slots[g_slot_count++];
    slot.meta = {name, ParamType::Float, description, min_value, max_value};
    slot.storage = StorageKind::Double;
    slot.value_ptr = value;
    return true;
}

bool ParamRegistry::register_param(const char *name,
                                   int32_t *value,
                                   int32_t min_value,
                                   int32_t max_value,
                                   const char *description)
{
    if (g_slot_count >= kMaxParams || value == nullptr || find_slot(name) != nullptr)
    {
        return false;
    }

    ParamSlot &slot = g_slots[g_slot_count++];
    slot.meta = {name, ParamType::Int32, description, static_cast<float>(min_value), static_cast<float>(max_value)};
    slot.storage = StorageKind::Int32;
    slot.value_ptr = value;
    return true;
}

bool ParamRegistry::register_param(const char *name,
                                   uint32_t *value,
                                   uint32_t min_value,
                                   uint32_t max_value,
                                   const char *description)
{
    if (g_slot_count >= kMaxParams || value == nullptr || find_slot(name) != nullptr)
    {
        return false;
    }

    ParamSlot &slot = g_slots[g_slot_count++];
    slot.meta = {name, ParamType::UInt32, description, static_cast<float>(min_value), static_cast<float>(max_value)};
    slot.storage = StorageKind::UInt32;
    slot.value_ptr = value;
    return true;
}

bool ParamRegistry::register_param(const char *name, bool *value, const char *description)
{
    if (g_slot_count >= kMaxParams || value == nullptr || find_slot(name) != nullptr)
    {
        return false;
    }

    ParamSlot &slot = g_slots[g_slot_count++];
    slot.meta = {name, ParamType::Bool, description, 0.0f, 1.0f};
    slot.storage = StorageKind::Bool;
    slot.value_ptr = value;
    return true;
}

const ParamEntry *ParamRegistry::find_param(const char *name) const
{
    const ParamSlot *slot = find_slot(name);
    return (slot != nullptr) ? &slot->meta : nullptr;
}

void ParamRegistry::list_params(const char *prefix) const
{
    for (size_t i = 0; i < g_slot_count; ++i)
    {
        const ParamSlot &slot = g_slots[i];
        if (!string_starts_with(slot.meta.name, prefix))
        {
            continue;
        }

        char value_text[64] = {0};
        format_value(slot, value_text, sizeof(value_text));

        if (slot.meta.type == ParamType::Bool)
        {
            write_line("%s = %s (%s) - %s",
                       slot.meta.name,
                       value_text,
                       type_name(slot.meta.type),
                       slot.meta.description);
        }
        else
        {
            write_line("%s = %s (%s, min=%.3f, max=%.3f) - %s",
                       slot.meta.name,
                       value_text,
                       type_name(slot.meta.type),
                       slot.meta.min_value,
                       slot.meta.max_value,
                       slot.meta.description);
        }
    }
}

void ParamRegistry::handle_command(const char *line)
{
    if (line == nullptr)
    {
        return;
    }

    char command_copy[kCommandBufferSize];
    std::snprintf(command_copy, sizeof(command_copy), "%s", line);

    char *cursor = command_copy;
    char *command = next_token(cursor);
    if (command == nullptr)
    {
        return;
    }

    if (string_equals(command, "help"))
    {
        write_line("RTT commands: help | list | get <name> | set <name> <value> | dump <prefix>");
        return;
    }

    if (string_equals(command, "list"))
    {
        list_params();
        return;
    }

    if (string_equals(command, "dump"))
    {
        char *prefix = next_token(cursor);
        list_params(prefix);
        return;
    }

    if (string_equals(command, "get"))
    {
        char *name = next_token(cursor);
        if (name == nullptr)
        {
            write_line("ERR missing parameter name");
            return;
        }

        const ParamSlot *slot = find_slot(name);
        if (slot == nullptr)
        {
            write_line("ERR unknown parameter: %s", name);
            return;
        }

        char value_text[64] = {0};
        format_value(*slot, value_text, sizeof(value_text));
        write_line("OK %s = %s", slot->meta.name, value_text);
        return;
    }

    if (string_equals(command, "set"))
    {
        char *name = next_token(cursor);
        char *value_text = next_token(cursor);
        if (name == nullptr || value_text == nullptr)
        {
            write_line("ERR usage: set <name> <value>");
            return;
        }

        ParamSlot *slot = find_slot_mutable(name);
        if (slot == nullptr)
        {
            write_line("ERR unknown parameter: %s", name);
            return;
        }

        char error_text[96] = {0};
        if (!set_value(*slot, value_text, error_text, sizeof(error_text)))
        {
            write_line("ERR %s: %s", slot->meta.name, error_text);
            return;
        }

        char current_value[64] = {0};
        format_value(*slot, current_value, sizeof(current_value));
        write_line("OK %s = %s", slot->meta.name, current_value);
        return;
    }

    write_line("ERR unknown command: %s", command);
}

void ParamRegistry::poll()
{
    char read_buffer[kReadBufferSize];
    const unsigned read_size = log().read(read_buffer, sizeof(read_buffer), 0);
    for (unsigned i = 0; i < read_size; ++i)
    {
        const char ch = read_buffer[i];
        if (ch == '\r' || ch == '\n')
        {
            if (g_command_length > 0)
            {
                g_command_buffer[g_command_length] = '\0';
                handle_command(g_command_buffer);
                g_command_length = 0;
            }
            continue;
        }

        if ((ch == '\b' || ch == 0x7F) && g_command_length > 0)
        {
            --g_command_length;
            continue;
        }

        if (g_command_length >= (kCommandBufferSize - 1))
        {
            g_command_length = 0;
            write_line("ERR command too long");
            continue;
        }

        g_command_buffer[g_command_length++] = ch;
    }
}

void ParamRegistry::init()
{
    if (g_initialized)
    {
        return;
    }

    g_slot_count = 0;
    g_command_length = 0;
    std::memset(g_command_buffer, 0, sizeof(g_command_buffer));
    register_default_params(*this);
    g_initialized = true;

    write_line("RTT debug service ready. Type 'help' for commands.");
}

ParamRegistry &registry()
{
    static ParamRegistry instance;
    return instance;
}

} // namespace HAL::LOGGER::RTTDebug

extern "C" void RTT_Debug_Init(void)
{
    HAL::LOGGER::RTTDebug::registry().init();
}

extern "C" void RTT_Debug_Poll(void)
{
    HAL::LOGGER::RTTDebug::registry().poll();
}

extern "C" void RTT_Debug_WriteString(const char *message)
{
    if (message == nullptr)
    {
        return;
    }

    HAL::LOGGER::Logger::getInstance().write_string(message);
}
