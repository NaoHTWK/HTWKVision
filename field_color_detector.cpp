#include <field_color_detector.h>

#include <cstdlib>
#include <cstring>

#include <ext_math.h>

using namespace ext_math;
using namespace std;

namespace htwk {

const int FieldColorDetector::pixelSpacing=16;	//nur jeden 16ten Pixel in x- und y-Richtung scannen
const int FieldColorDetector::minFieldArea=150;
const int FieldColorDetector::colorBorder=8;

FieldColorDetector::FieldColorDetector(int _width, int _height, int *_lutCb, int *_lutCr):
		width(_width),height(_height),lutCb(_lutCb),lutCr(_lutCr){
	greenCy=0;
	greenCr=0;
	greenCb=0;
	resetArrays();
}

FieldColorDetector::~FieldColorDetector(){
}

/**
 * detects the yCbCr color of the playing field in the image.
 * saves two histograms with rating values for different color combinations
 */
void FieldColorDetector::proceed(const uint8_t * const img) {
	resetArrays();
	searchInitialSeed(img);

	for(int y=0;y<height;y+=pixelSpacing){
		for(int x=0;x<width;x+=pixelSpacing){
			int cy=((getY(img,x,y)-seedY)>>2)+HIST_SIZE/2;
			if(cy<0||cy>=HIST_SIZE)continue;
			int cb=((getCb(img,x,y)-seedCb)>>1)+HIST_SIZE/2;
			if(cb<0||cb>=HIST_SIZE)continue;
			int cr=((getCr(img,x,y)-seedCr)>>1)+HIST_SIZE/2;
			if(cr<0||cr>=HIST_SIZE)continue;
			histYCbCr[cy+cb*HIST_SIZE+cr*HIST_SIZE*HIST_SIZE]++;
		}
	}

	smoothHist(histYCbCr,histYCbCrSmooth);
	smoothHist(histYCbCrSmooth,histYCbCrSmooth2);

	for(int cy=0;cy<HIST_SIZE;cy++){
		for(int cb=0;cb<HIST_SIZE;cb++){
			for(int cr=0;cr<HIST_SIZE;cr++){
				int addr=cy+cb*HIST_SIZE+cr*HIST_SIZE*HIST_SIZE;
				histYCbCrSmooth[addr]-=(histYCbCrSmooth2[addr]>>6);
			}
		}
	}

	createLUT(histYCbCrSmooth);
}

void FieldColorDetector::createLUT(const int* const hist){
		int mCy=HIST_SIZE/2;
		int mCb=HIST_SIZE/2;
		int mCr=HIST_SIZE/2;
		for(int i=0;i<HIST_SIZE/3;i++){
			int h=hist[mCy+mCb*HIST_SIZE+mCr*HIST_SIZE*HIST_SIZE];
			int maxH=h;
			int bestCy=0;
			int bestCb=0;
			int bestCr=0;
			for(int dCy=-2;dCy<=2;dCy++){
				for(int dCb=-2;dCb<=2;dCb++){
					for(int dCr=-2;dCr<=2;dCr++){
						int ny=mCy+dCy;
						if(ny<0||ny>=HIST_SIZE)continue;
						int nb=mCb+dCb;
						if(nb<0||nb>=HIST_SIZE)continue;
						int nr=mCr+dCr;
						if(nr<0||nr>=HIST_SIZE)continue;
						int ch=hist[ny+nb*HIST_SIZE+nr*HIST_SIZE*HIST_SIZE];
						if(ch>maxH){
							bestCy=ny;
							bestCb=nb;
							bestCr=nr;
							maxH=ch;
						}
					}
				}
			}
			if(maxH==h)break;
			mCy=bestCy;
			mCb=bestCb;
			mCr=bestCr;
		}

		greenCy=clamp(colorBorder,seedY+((mCy-HIST_SIZE/2)<<2),255-colorBorder);
		greenCb=clamp(colorBorder,seedCb+((mCb-HIST_SIZE/2)<<1),255-colorBorder);
		greenCr=clamp(colorBorder,seedCr+((mCr-HIST_SIZE/2)<<1),255-colorBorder);
		vector<color> s;
		vector<color> entries;
		{
			color c;
			c.cy=mCy;
			c.cb=mCb;
			c.cr=mCr;
			s.push_back(c);
		}
		lut[mCy+mCb*HIST_SIZE+mCr*HIST_SIZE*HIST_SIZE]=1;
		while(!s.empty()){
			color p=s.front();
			s.erase(s.begin());
			entries.push_back(p);
			for(int dCy=-1;dCy<=1;dCy++){
				for(int dCb=-1;dCb<=1;dCb++){
					for(int dCr=-1;dCr<=1;dCr++){
						if(abs(dCy)+abs(dCb)+abs(dCr)>1)continue;
						int ny=p.cy+dCy;
						if(ny<0||ny>=HIST_SIZE)continue;
						int nb=p.cb+dCb;
						if(nb<0||nb>=HIST_SIZE)continue;
						int nr=p.cr+dCr;
						if(nr<0||nr>=HIST_SIZE)continue;
						int addr=ny+nb*HIST_SIZE+nr*HIST_SIZE*HIST_SIZE;
						if(lut[addr]>0)continue;
						int nh=hist[addr];
						if(nh>0){
							{
								color c;
								c.cy=ny;
								c.cb=nb;
								c.cr=nr;
								s.push_back(c);
							}
							lut[ny+nb*HIST_SIZE+nr*HIST_SIZE*HIST_SIZE]=1;
						}
					}
				}
			}
		}
		smoothLUT(entries,2);
		smoothLUT(entries,2);
	}

void FieldColorDetector::smoothLUT(vector<color> &entries, int r){
	vector<color> entries2;
	for(const color &c : entries){
		entries2.push_back(c);
	}
	for(const color &c : entries2){
		for(int dCy=-1;dCy<=1;dCy++){
			for(int dCb=-1;dCb<=1;dCb++){
				for(int dCr=-1;dCr<=1;dCr++){
					if(abs(dCy)+abs(dCb)+abs(dCr)>r)continue;
					int ny=c.cy+dCy;
					if(ny<0||ny>=HIST_SIZE)continue;
					int nb=c.cb+dCb;
					if(nb<0||nb>=HIST_SIZE)continue;
					int nr=c.cr+dCr;
					if(nr<0||nr>=HIST_SIZE)continue;
					int addr=ny+nb*HIST_SIZE+nr*HIST_SIZE*HIST_SIZE;
					if(lut[addr]>0)continue;
					lut[addr]=1;
					{
						color c;
						c.cy=ny;
						c.cb=nb;
						c.cr=nr;
						entries.push_back(c);
					}
				}
			}
		}
	}
}

void FieldColorDetector::smoothHist(const int* const histSrc, int* histDest){
	int t=4;
	int rCy=2;
	int rCb=2;
	int rCr=1;
	for(int cy=rCy;cy<HIST_SIZE-rCy;cy++){
		for(int cb=rCb;cb<HIST_SIZE-rCb;cb++){
			for(int cr=rCr;cr<HIST_SIZE-rCr;cr++){
				int c=histSrc[cy+cb*HIST_SIZE+cr*HIST_SIZE*HIST_SIZE];
				if(c>t){
					for(int dCy=-rCy;dCy<=rCy;dCy++){
						for(int dCb=-rCb;dCb<=rCb;dCb++){
							for(int dCr=-rCr;dCr<=rCr;dCr++){
								int ny=cy+dCy;
								int nb=cb+dCb;
								int nr=cr+dCr;
								histDest[ny+nb*HIST_SIZE+nr*HIST_SIZE*HIST_SIZE]+=c;
							}
						}
					}

				}
			}
		}
	}
}

void FieldColorDetector::searchInitialSeed(const uint8_t * const img){

	//building histogram of all cr-channel
	int seedSearchBorder=width/16;
	for(int y=seedSearchBorder;y<height-1-seedSearchBorder;y+=pixelSpacing){
		for(int x=seedSearchBorder;x<width-seedSearchBorder;x+=pixelSpacing){
			int cr=getCr(img,x,y);
			histCr[cr]++;
		}
	}

	//finding initial cr-value (later used as a seed color)
	seedCr=clamp(colorBorder,getStableMin(histCr,minFieldArea),255-colorBorder);

	//build histogram of cb-channel for promising pixels
	for(int y=0;y<height-1;y+=pixelSpacing){
		for(int x=0;x<width;x+=pixelSpacing){
			int cr=getCr(img,x,y);
			if(abs(cr-seedCr)<4){
				int cb=getCb(img,x,y);
				histCb[cb]++;
			}
		}
	}
	//finding initial cb-value (later used as a seed color)
	seedCb=clamp(colorBorder,getPeak(histCb),255-colorBorder);

	//build histogram of y-channel for promising pixels
	for(int y=0;y<height-1;y+=pixelSpacing){
		for(int x=0;x<width;x+=pixelSpacing){
			int cr=getCr(img,x,y);
			if(abs(cr-seedCr)<8){
				int cb=getCb(img,x,y);
				if(abs(cb-seedCb)<8){
					int cy=getY(img,x,y);
					histY[cy]++;
				}
			}
		}
	}
	//finding initial y-value (later used as a seed color)
	seedY=clamp(colorBorder,getPeak(histY),255-colorBorder);

}

void FieldColorDetector::resetArrays() {
    memset(histY,0,sizeof(histY));
    memset(histCb,0,sizeof(histCb));
    memset(histCr,0,sizeof(histCr));
    memset(lut,0,sizeof(lut));
    memset(histYCbCr,0,sizeof(histYCbCr));
    memset(histYCbCrSmooth,0,sizeof(histYCbCrSmooth));
    memset(histYCbCrSmooth2,0,sizeof(histYCbCrSmooth2));
}

/**
 * only in a histogram with 256 bins: get the index of the bin with the lowest value but with a stabilization criteria ('thres').
 */

int FieldColorDetector::getStableMin(const int* const hist, int thres) {
	int sum=0;
	for(int i=0;i<256;i++){
		sum+=hist[i];
		if(sum>thres){
			return i;
		}
	}
	return 0;
}

/**
 * only in a histogram with 256 bins: get the index of the bin with the highest value
 */
int FieldColorDetector::getPeak(const int* const hist) {
	int max=0;
	int maxIdx=0;
	for(int i=0;i<256;i++){
		if(hist[i]>max){
			max=hist[i];
			maxIdx=i;
		}
	}
	return maxIdx;
}

/**
 * test, if a given yuv-color matches the field-color (used to detect all pixels on the carpet)
 */
bool FieldColorDetector::isGreen(int cyReal, int cbReal, int crReal) const {
	int cy=((cyReal-seedY)>>2)+HIST_SIZE/2;
	if(cy<0||cy>=HIST_SIZE)return false;
	int cb=((cbReal-seedCb)>>1)+HIST_SIZE/2;
	if(cb<0||cb>=HIST_SIZE)return false;
	int cr=((crReal-seedCr)>>1)+HIST_SIZE/2;
	if(cr<0||cr>=HIST_SIZE)return false;
	if(lut[cy+cb*HIST_SIZE+cr*HIST_SIZE*HIST_SIZE]>0){
		return true;
	}
	return false;
}

color FieldColorDetector::getColor() const {
	color c;
	c.cy=greenCy;
	c.cb=greenCb;
	c.cr=greenCr;
	return c;
}

}  // namespace htwk
