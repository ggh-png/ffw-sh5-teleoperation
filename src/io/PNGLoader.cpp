#include "PNGLoader.hpp"
#include <zlib.h>
#include <fstream>
#include <cstring>
#include <cstdio>
#include <cmath>

static uint32_t be32(const uint8_t* p) {
    return (uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3];
}

// PNG Paeth predictor
static uint8_t paeth(uint8_t a, uint8_t b, uint8_t c) {
    int p  = int(a)+int(b)-int(c);
    int pa = std::abs(p-int(a));
    int pb = std::abs(p-int(b));
    int pc = std::abs(p-int(c));
    if(pa<=pb && pa<=pc) return a;
    if(pb<=pc)           return b;
    return c;
}

RGBAPixmap PNGLoader::load(const std::string& path) {
    RGBAPixmap res;

    std::ifstream f(path, std::ios::binary);
    if(!f) { fprintf(stderr,"[PNGLoader] Cannot open '%s'\n",path.c_str()); return res; }
    std::vector<uint8_t> buf(std::istreambuf_iterator<char>(f), {});
    f.close();

    // PNG signature
    static const uint8_t kSig[8] = {0x89,0x50,0x4E,0x47,0x0D,0x0A,0x1A,0x0A};
    if(buf.size()<8 || memcmp(buf.data(),kSig,8)!=0) {
        fprintf(stderr,"[PNGLoader] Not a PNG: '%s'\n",path.c_str()); return res;
    }

    int width=0,height=0,bitDepth=0,colorType=0,interlace=0;
    std::vector<uint8_t> compressed;

    size_t pos=8;
    while(pos+12<=buf.size()) {
        uint32_t len  = be32(buf.data()+pos); pos+=4;
        char     type[5]={0}; memcpy(type,buf.data()+pos,4); pos+=4;
        const uint8_t* data=buf.data()+pos;

        if     (strcmp(type,"IHDR")==0 && len>=13) {
            width    = (int)be32(data);
            height   = (int)be32(data+4);
            bitDepth = data[8];
            colorType= data[9];
            interlace= data[12];
        } else if(strcmp(type,"IDAT")==0) {
            compressed.insert(compressed.end(), data, data+len);
        } else if(strcmp(type,"IEND")==0) {
            break;
        }
        pos += len + 4; // data + CRC
    }

    if(width<=0||height<=0||bitDepth!=8||(colorType!=2&&colorType!=6)||interlace!=0) {
        fprintf(stderr,"[PNGLoader] Unsupported PNG (w=%d h=%d bd=%d ct=%d il=%d): '%s'\n",
                width,height,bitDepth,colorType,interlace,path.c_str());
        return res;
    }

    int channels  = (colorType==6) ? 4 : 3;
    int rawStride = 1 + width*channels; // filter byte + row pixels
    std::vector<uint8_t> raw(height * rawStride, 0);

    // Decompress IDAT stream
    uLongf rawLen = (uLongf)raw.size();
    int zret = uncompress(raw.data(), &rawLen,
                          compressed.data(), (uLong)compressed.size());
    if(zret != Z_OK) {
        fprintf(stderr,"[PNGLoader] zlib uncompress failed (%d) for '%s'\n",zret,path.c_str());
        return res;
    }

    res.width  = width;
    res.height = height;
    res.rgba.resize(width * height * 4, 255);

    for(int y=0; y<height; ++y) {
        uint8_t* row  = raw.data() + y * rawStride;
        uint8_t  flt  = row[0];
        row++;  // advance past filter byte
        const uint8_t* prev = (y>0) ? (raw.data()+(y-1)*rawStride+1) : nullptr;
        int bpp = channels;

        // Reconstruct scanline in-place
        for(int i=0; i<width*channels; ++i) {
            uint8_t a = (i >= bpp)   ? row[i-bpp]   : 0;
            uint8_t b = prev          ? prev[i]       : 0;
            uint8_t c = (prev&&i>=bpp)? prev[i-bpp]  : 0;
            switch(flt) {
                case 0: break;
                case 1: row[i] = row[i] + a;               break;
                case 2: row[i] = row[i] + b;               break;
                case 3: row[i] = row[i] + (uint8_t)((int(a)+int(b))/2); break;
                case 4: row[i] = row[i] + paeth(a,b,c);   break;
            }
        }

        // Copy to RGBA output (flip Y so texture origin is bottom-left)
        int dstY = height - 1 - y;
        uint8_t* dst = res.rgba.data() + dstY * width * 4;
        if(channels == 4) {
            memcpy(dst, row, (size_t)width*4);
        } else {
            for(int x=0; x<width; ++x) {
                dst[x*4+0] = row[x*3+0];
                dst[x*4+1] = row[x*3+1];
                dst[x*4+2] = row[x*3+2];
                dst[x*4+3] = 255;
            }
        }
    }

    fprintf(stderr,"[PNGLoader] Loaded '%s' %dx%d ch=%d\n",
            path.c_str(), width, height, channels);
    return res;
}
