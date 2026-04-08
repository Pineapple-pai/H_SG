#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
namespace HAL::LOGGER::RTTDebug
{
enum class ParamType : uint8_t
{
    Float,
    Int32,
    UInt32,
    Bool
};

struct ParamEntry
{
    const char *name;
    ParamType type;
    const char *description;
    float min_value;
    float max_value;
};

class ParamRegistry
{
  public:
    bool register_param(const char *name, float *value, float min_value, float max_value, const char *description);
    bool register_param(const char *name, double *value, float min_value, float max_value, const char *description);
    bool register_param(const char *name, int32_t *value, int32_t min_value, int32_t max_value, const char *description);
    bool register_param(const char *name, uint32_t *value, uint32_t min_value, uint32_t max_value, const char *description);
    bool register_param(const char *name, bool *value, const char *description);

    const ParamEntry *find_param(const char *name) const;
    void list_params(const char *prefix = nullptr) const;
    void handle_command(const char *line);
    void poll();
    void init();
};

ParamRegistry &registry();
} // namespace HAL::LOGGER::RTTDebug
#endif

#ifdef __cplusplus
extern "C" {
#endif

void RTT_Debug_Init(void);
void RTT_Debug_Poll(void);
void RTT_Debug_WriteString(const char *message);

#ifdef __cplusplus
}
#endif
