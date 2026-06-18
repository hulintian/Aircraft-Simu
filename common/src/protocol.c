/** @file protocol.c
 *  @brief 协议报文头构造实现。
 */
#include "common/protocol.h"
#include "common/build_info.h"

#include <stddef.h>

/** @brief 构造统一报文头。 */
PacketHeader packet_header_make(
    PacketType type,
    uint32_t instance_id,
    uint32_t seq,
    double sim_time,
    uint32_t payload_size,
    uint32_t payload_crc32)
{
    PacketHeader header;

    header.magic = SIM_PACKET_MAGIC;
    header.version_major = MISSILE_SIM_PROTOCOL_VERSION_MAJOR;
    header.version_minor = MISSILE_SIM_PROTOCOL_VERSION_MINOR;
    header.type = (uint16_t)type;
    header.header_size = (uint16_t)sizeof(PacketHeader);
    header.instance_id = instance_id;
    header.seq = seq;
    header.sim_time = sim_time;
    header.payload_size = payload_size;
    header.payload_crc32 = payload_crc32;

    return header;
}
