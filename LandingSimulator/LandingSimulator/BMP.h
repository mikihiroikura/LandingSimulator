#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <gl/GL.h>

//BitmapHeader構造体
typedef struct _BitmapHeaders{
	char	filetype1;
	char	filetype2;
	int		filesize;
	short	reserve1;
	short	reserve2;
	int		offset;

	int		header;
	int		width;
	int		height;
	short	planes;
	short	bit_count;
	int		compression;
	int		size_image;
	int		x_resolution;
	int		y_resolution;
	int		clr_used;
	int		clr_important;
}BitmapHeaders;

//ODE描画ピクセルデータをＢＭＰファイル出力（１コマンド＝1ファイル）
extern int WriteBMP(int width, int height);
extern int SaveBMP(char *folder,int width, int height);
