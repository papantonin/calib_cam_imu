// Small synthetic fixture generator using the pinned upstream tag bitmap definition.
#include <apriltag.h>
#include <tag36h11.h>
#include <tagStandard41h12.h>
#include <png.h>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
int main(int argc,char** argv) {
  if(argc!=8 && argc!=9) { std::cerr<<"output family id rotation width height scale [degraded]\n"; return 2; }
  const std::string family=argv[2];
  const int id=std::stoi(argv[3]), rotation=std::stoi(argv[4]), width=std::stoi(argv[5]), height=std::stoi(argv[6]), scale=std::stoi(argv[7]);
  if(family!="tag36h11" && family!="tagStandard41h12") return 2;
  std::unique_ptr<apriltag_family_t,void(*)(apriltag_family_t*)> tf(family=="tag36h11"?tag36h11_create():tagStandard41h12_create(),family=="tag36h11"?tag36h11_destroy:tagStandard41h12_destroy);
  if(id<0 || unsigned(id)>=tf->ncodes || scale<1 || width<1 || height<1 || rotation<0 || rotation>3) return 2;
  std::unique_ptr<image_u8_t,void(*)(image_u8_t*)> tag(apriltag_to_image(tf.get(),id),image_u8_destroy);
  const bool legacy=argc==9 && std::string(argv[8])=="legacy";
  if(legacy && family!="tag36h11") return 2;
  const int bitmap_width=tag->width+(legacy?2:0);
  std::vector<unsigned char> bitmap(std::size_t(bitmap_width)*bitmap_width,255);
  for(int y=0;y<bitmap_width;++y) for(int x=0;x<bitmap_width;++x) {
    if(!legacy) bitmap[std::size_t(y)*bitmap_width+x]=tag->buf[y*tag->stride+x];
    else if(x>0 && y>0 && x<bitmap_width-1 && y<bitmap_width-1) {
      // Independent fixture: 6x6 payload copied inside a 2-cell black border.
      bitmap[std::size_t(y)*bitmap_width+x]=(x>=3 && x<9 && y>=3 && y<9)?tag->buf[(y-1)*tag->stride+x-1]:0;
    }
  }
  const int side=bitmap_width*scale, x0=(width-side)/2, y0=(height-side)/2;
  if(x0<20 || y0<20) return 2;
  std::vector<unsigned char> pixels(std::size_t(width)*height,255);
  for(int y=0;y<side;++y) for(int x=0;x<side;++x) {
    int xx=x,yy=y;
    for(int r=0;r<rotation;++r) { const int old=xx; xx=side-1-yy; yy=old; }
    pixels[std::size_t(y0+yy)*width+x0+xx]=bitmap[std::size_t(y/scale)*bitmap_width+x/scale];
  }
  if(argc==9 && std::string(argv[8])=="degraded") {
    // Independent 3x3 box blur and fixed-seed bounded intensity noise.
    const auto original=pixels;
    unsigned state=1234567;
    for(int y=1;y<height-1;++y) for(int x=1;x<width-1;++x) {
      unsigned total=0;
      for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) total+=original[std::size_t(y+dy)*width+x+dx];
      state=1664525u*state+1013904223u;
      const int value=static_cast<int>(total/9)+static_cast<int>((state>>16)%17)-8;
      pixels[std::size_t(y)*width+x]=static_cast<unsigned char>(std::clamp(value,0,255));
    }
  }
  png_image image{}; image.version=PNG_IMAGE_VERSION; image.width=width; image.height=height; image.format=PNG_FORMAT_GRAY;
  const bool ok=png_image_write_to_file(&image,argv[1],0,pixels.data(),0,nullptr);
  if(!ok) std::cerr<<image.message<<'\n';
  png_image_free(&image);
  return ok?0:1;
}
