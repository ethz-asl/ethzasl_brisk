/*
 * falsecolor.h
 *
 *  Created on: Oct 19, 2012
 *      Author: slynen
 */

#ifndef FALSECOLOR_H_
#define FALSECOLOR_H_

#define DEG2RAD 0.01745329

struct color{
	unsigned char rgbBlue;
	unsigned char rgbGreen;
	unsigned char rgbRed;
	color(){
		rgbBlue = rgbGreen = rgbRed = 0;
	}
};

struct palette{
	enum palettetypes{
		Linear_red_palettes,
		GammaLog_red_palettes,
		Inversion_red_palette,
		Linear_palettes,
		GammaLog_palettes,
		Inversion_palette,
		False_color_palette1,
		False_color_palette2,
		False_color_palette3
	};
	color colors[256];
};


palette GetPalette(palette::palettetypes pal);

#undef DEG2RAD

void convertFalseColor(const cv::Mat& srcmat, cv::Mat& dstmat, palette::palettetypes paltype);

#endif /* FALSECOLOR_H_ */
