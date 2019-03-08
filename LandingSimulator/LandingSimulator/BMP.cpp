#include "BMP.h"
#include <vector>

using namespace std;

GLubyte * pixel_data_logs[600];
char bmpfile[10] = "000.bmp";	// �u000�`999�v�̘A�ԂƂȂ�t�@�C��
int cnt = 0;


// BMP�w�b�_�̏����l
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

// BMP�w�b�_�̏���
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

// ODE�`��f�[�^��BMP�t�@�C���ւ̏���
int WriteBMP(int width, int height){
	GLubyte *ode_pixel_data;

	// �������̈�m��
	ode_pixel_data = (GLubyte*)malloc((width*3)*(height)*(sizeof(GLubyte)));
	glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,ode_pixel_data);
	
	//vector�ɕۑ�
	pixel_data_logs[cnt]= ode_pixel_data;
	cnt++;

	return 0;
}

//�A�Ԃ�BMP�t�@�C���̕ۑ�
int SaveBMP(int width, int height) {
	FILE *fp;
	BitmapHeaders file;
	int x, y;
	
	// �t�@�C���I�[�v��
	fp = fopen(bmpfile, "wb");
	if (fp == NULL) {
		printf("failure\n");
		return -1;
	}

	// BMP�w�b�_�̏�����
	InitHeaders(&file);

	// BMP�w�b�_�̏���
	file.width = width;
	file.height = height;
	file.filesize = (width * 3)*height + 54;
	WriteHeaders(&file, fp);

	// BMP�s�N�Z���f�[�^�̏���
	for (size_t i = 0; i < 600; i++)
	{
		for (y = 0; y < height; y++) {
			for (x = 0; x < width; x++) {
				fwrite((pixel_data_logs[i] + x * 3 + (width * 3)*y + 2), sizeof(GLubyte), 1, fp);
				fwrite((pixel_data_logs[i] + x * 3 + (width * 3)*y + 1), sizeof(GLubyte), 1, fp);
				fwrite((pixel_data_logs[i] + x * 3 + (width * 3)*y + 0), sizeof(GLubyte), 1, fp);
			}
		}
	}

	// �t�@�C������A�Ԃɂ���
	bmpfile[2]++;
	if (bmpfile[2] == '9' + 1) {
		bmpfile[2] = '0';
		bmpfile[1]++;
		if (bmpfile[1] == '9' + 1) {
			bmpfile[1] = '0';
			bmpfile[0]++;
		}
	}

	// �t�@�C���N���[�Y
	fclose(fp);
}