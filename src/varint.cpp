// Implementation of the varint integer encoding. Encodes a 32-bit signed int in 1..5 bytes.
#include <stdio.h>
#include <stdint.h>

// encodeVarint encodes an array of signed ints into a buffer of varint bytes. Returns the number
// of bytes produced. Returns 0 if the encoding ran out of space.
// Reference: http://jeelabs.org/article/1620c/
int encodeVarint(int32_t ints[], int count, uint8_t *buf, int len) {
    int len0 = len; // remember original length
    while (count--) {
        int32_t v = *ints++; // value to encode
        // special-case zero value
        if (v == 0) {
            if (len < 1) return 0; // out of space
            *buf++ = 0x80;
            len--;
            continue;
        }

        // handle negative values
        uint32_t u = v << 1; // make space for sign in bottom bit
        if (v < 0) u = ~u; // encode positive value, place sign in bottom bit

        // generate encoding
        const int max = 5; // produce max 5 bytes
        uint8_t temp[max]; // produce output backwards
        int i = max;
        while (u != 0) {
            temp[--i] = u & 0x7f;
            u = u >> 7;
        }
        temp[4] |= 0x80; // bit to signal last byte

        if (max-i > len) return 0; // out of space
        len -= max-i;
        while (i < max) *buf++ = temp[i++];
    }
    return len0-len;
}

// decodeVarint decodes a single varint from a buffer and stores it at the value ptr.
// Returns the number of bytes consumed from the buffer or 0 if no varint was decoded.
// Reference: http://jeelabs.org/article/1620c/
int decodeVarint(uint8_t buf[], int len, int32_t *value) {
    uint32_t val = 0; // producing int32_t in the end but shifts are easier with uint32_t
    for (int l=0; l<len; l++) {
        val = (val << 7) | (uint32_t)(*buf & 0x7f);
        if ((*buf & 0x80) != 0) {
            // last byte flag set, output an int
            if ((val & 1) == 0) val = val >> 1; // positive value, eat sign bit
            else val = ~(val >> 1); // negative value, eat sign bit and negate
            *value = (int32_t)val;
            return l+1;
        }
        buf++;
    }
    // did not get to the end of a value
    return 0;
}

// decodeVarints decodes buffer of varint bytes into an array of signed ints. Returns the number of
// ints decoded or 0 if the decoding ran out of space.
// Reference: http://jeelabs.org/article/1620c/
int decodeVarints(uint8_t buf[], int len, int32_t *ints, int count) {
    int c = 0; // ints produced
    while (len > 0) {
        int32_t v;
        int l = decodeVarint(buf, len, &v);
        //printf("decodeVarints: len=%d l=%d count=%d c=%d v=%d\n", len, l, count, c, v);
        if (l == 0) return c; // no more varint, return what we already have
        if (c == count) return 0; // out of space
        ints[c++] = v;
        buf += l;
        len -= l;
    }
    return c; // reached end of buf, return what we found
}
