/** @file provenance.c
 *  @brief 运行证据与 JSON 清单公共实现。
 */
#define _POSIX_C_SOURCE 200809L

#include "common/provenance.h"

#include "common/crc32.h"

#include <time.h>

SimStatus provenance_file_crc32(
    const char *path,
    uint32_t *crc_out,
    uint64_t *size_out)
{
    unsigned char buffer[16384];
    uint32_t state = crc32_init();
    uint64_t total_size = 0u;
    FILE *file;

    if (path == 0 || crc_out == 0 || size_out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    file = fopen(path, "rb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    for (;;) {
        const size_t got = fread(buffer, 1u, sizeof(buffer), file);

        if (got > 0u) {
            state = crc32_update(state, buffer, got);
            total_size += (uint64_t)got;
        }
        if (got != sizeof(buffer)) {
            if (ferror(file) != 0) {
                (void)fclose(file);
                return SIM_ERR_IO;
            }
            break;
        }
    }
    if (fclose(file) != 0) {
        return SIM_ERR_IO;
    }
    *crc_out = crc32_finalize(state);
    *size_out = total_size;
    return SIM_OK;
}

SimStatus provenance_write_json_string(FILE *file, const char *value)
{
    const unsigned char *cursor = (const unsigned char *)value;

    if (file == 0 || value == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (fputc('"', file) == EOF) {
        return SIM_ERR_IO;
    }
    while (*cursor != '\0') {
        const unsigned char ch = *cursor++;

        if (ch == '"' || ch == '\\') {
            if (fputc('\\', file) == EOF || fputc((int)ch, file) == EOF) {
                return SIM_ERR_IO;
            }
        } else if (ch == '\b' || ch == '\f' || ch == '\n' || ch == '\r' || ch == '\t') {
            const char escape = ch == '\b' ? 'b' :
                (ch == '\f' ? 'f' : (ch == '\n' ? 'n' : (ch == '\r' ? 'r' : 't')));

            if (fputc('\\', file) == EOF || fputc(escape, file) == EOF) {
                return SIM_ERR_IO;
            }
        } else if (ch < 0x20u) {
            if (fprintf(file, "\\u%04x", (unsigned int)ch) < 0) {
                return SIM_ERR_IO;
            }
        } else if (fputc((int)ch, file) == EOF) {
            return SIM_ERR_IO;
        }
    }
    return fputc('"', file) == EOF ? SIM_ERR_IO : SIM_OK;
}

void provenance_format_wall_clock_utc(char *out, size_t out_size)
{
    time_t now;
    struct tm utc;

    if (out == 0 || out_size == 0u) {
        return;
    }
    now = time(0);
    if (now == (time_t)-1 || gmtime_r(&now, &utc) == 0 ||
        strftime(out, out_size, "%Y-%m-%dT%H:%M:%SZ", &utc) == 0u) {
        (void)snprintf(out, out_size, "unknown");
    }
}
