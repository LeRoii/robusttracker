
inline const int GetMsgLen(uint8_t u8)
{
    // printf("us:%d\n", u8);
    return u8 & 0x3F;
}

uint8_t viewlink_protocal_checksum(uint8_t *buf)
{
    uint8_t len = GetMsgLen(buf[3]);
    uint8_t checksum = buf[3];
    for (uint8_t i = 0; i < len - 2; i++)
    {
        // printf("checksum:%#x\n", checksum);
        checksum = checksum ^ buf[4 + i];
    }
    return (checksum);
}

int bufferParsing(uint8_t *buf, int &trackMissX, int &trackMissY)
{

    const std::vector<uint8_t> frameStart = {0x55, 0xAA, 0xDC};
    if (buf[0] == frameStart[0] && buf[1] == frameStart[1] && buf[2] == frameStart[2])
    {
        if (buf[14] == viewlink_protocal_checksum(buf))
        {
            trackMissX = (buf[5] << 8) ^ buf[6];
            trackMissY = (buf[7] << 8) ^ buf[8];
            return 0;
        }
        return -1;
    }

    return -1;
}
