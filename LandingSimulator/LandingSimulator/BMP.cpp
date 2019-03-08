#include "BMP.h"
#include <vector>
#include <cstdio>

using namespace std;

#define FILE_NAME "/%d.bmp"

vector<GLubyte *> pixel_data_logs;
char bmpfile[10];	// 「000～999」の連番となるファイル
char bmpfile_name[256];



// BMPヘッダの初期値
void InitHeaders(BitmapHeaders *file){
	file->filetype1		='B';
	file->filetype2		='M';
	file->filesize		= 0;
	file->reserve1		= 0;
	file->reserve2		= 0;
	file->offset		= 54;

	file->header		=40;
	file->width			= 0;
	file->height		= 0;
	file->planes		= 1;
	file->bit_count		=24;
	file->compression	= 0;
	file->size_image	= 0;
	file->x_resolution	= 0;
	file->y_resolution	= 0;
	file->clr_used		= 0;
	file->clr_important	= 0;
};

// BMPヘッダの書込
void WriteHeaders(BitmapHeaders *file,FILE *fp){
	fwrite(&(file->filetype1), sizeof(char),1,fp);
	fwrite(&(file->filetype2), sizeof(char),1,fp);
	fwrite(&(file->filesize), sizeof(int),1,fp);
	fwrite(&(file->reserve1), sizeof(short),1,fp);
	fwrite(&(file->reserve2), sizeof(short),1,fp);
	fwrite(&(file->offset), sizeof(int),1,fp);

	fwrite(&(file->header), sizeof(int),1,fp);
	fwrite(&(file->width), sizeof(int),1,fp);
	fwrite(&(file->height), sizeof(int),1,fp);
	fwrite(&(file->planes), sizeof(short),1,fp);
	fwrite(&(file->bit_count), sizeof(short),1,fp);
	fwrite(&(file->compression), sizeof(int),1,fp);
	fwrite(&(file->size_image), sizeof(int),1,fp);
	fwrite(&(file->x_resolution), sizeof(int),1,fp);
	fwrite(&(file->y_resolution), sizeof(int),1,fp);
	fwrite(&(file->clr_used), sizeof(int),1,fp);
	fwrite(&(file->clr_important), sizeof(int),1,fp);
}

// ODE描画データのBMPファイルへの書込
int WriteBMP(int width, int height){
	GLubyte *ode_pixel_data;

	// メモリ領域確保
	ode_pixel_data = (GLubyte*)malloc((width*3)*(height)*(sizeof(GLubyte)));
	glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,ode_pixel_data);
	
	//vectorに保存
	pixel_data_logs.push_back(ode_pixel_data);

	return 0;
}

//連番でBMPファイルの保存
int SaveBMP(char *folder,int width, int height) {
	FILE *fp;
	BitmapHeaders file;
	int x, y;

	// BMPヘッダの初期化
	InitHeaders(&file);

	// BMPピクセルデータの書込
	for (int i = 0; i < pixel_data_logs.size(); i++)
	{
		//ファイル名指定
		sprintf(bmpfile, FILE_NAME, i);
		sprintf(bmpfile_name, "%s%s", folder, bmpfile);

		// ファイルオープン
		fp = fopen(bmpfile_name, "wb");
		if (fp == NULL) {
			printf("failure\n");
			return -1;
		}
		// BMPヘッダの書込
		file.width = width;
		file.height = height;
		file.filesize = (width * 3)*height + 54;
		WriteHeaders(&file, fp);
		for (y = 0; y < height; y++) {
			for (x = 0; x < width; x++) {
				fwrite((pixel_data_logs[i] + x * 3 + (width * 3)*y + 2), sizeof(GLubyte), 1, fp);
				fwrite((pixel_data_logs[i] + x * 3 + (width * 3)*y + 1), sizeof(GLubyte), 1, fp);
				fwrite((pixel_data_logs[i] + x * 3 + (width * 3)*y + 0), sizeof(GLubyte), 1, fp);
			}
		}

		printf("img No.%d saved.\n", i);

		delete pixel_data_logs[i];


		// ファイルクローズ
		fclose(fp);
	}

	
}