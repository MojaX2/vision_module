
/**********************************************************
Segmentation.h			Copyright 2015.10.02 by N.Ikeda

�Z�O�����e�[�V�����v���O����
���ߖT�ł̃Z�O�����e�[�V�������s���܂�

�g�p���̒���
���̓}�b�v���ɂ�startNum�ȏ�̒l�������Ă͂����܂���B
����ĕK��startNum�̒l��map���̒l�ȏ�̂��̂��g������
�܂��A���̓}�b�v�̃f�t�H���g�l��-1�ŁA���͒l��1�ȏ�

**********************************************************/

/*�}�b�v�̕��A�����A�}�b�v�̃|�C���^�A�Z�O�����g���������̈�ԍ��A�Z�O�����g�ԍ��A�Ō�̃Z�O�����g�ԍ�
num�������Z�O�����e�[�V�������A�e�Z�O�����g��startNum����n�܂�Z�O�����g�ԍ���
���蓖�Ă�BendNum�ɂ͍Ō�̃Z�O�����g�ԍ�����������*/
void segmentate(const int &width, const int &height, 
				int *map, const int &num, const int &startNum, int &endNum);
//�r�c�@(������)
void segmentIkedaSearch(const int &width, const int &height, 
				int *map, const int &num, const int &startNum, int &endNum);
//�̈�g���@(4�ߖT)
void segmentRegionGrow(const int &width, const int &height, 
				int *map, const int &num, const int &startNum, int &endNum);

//�Z�O�����g�̖ʐς𓾂�
int getSegmentArea(const int &reso, const int &num, int *map);

void numset(int *list, const int &num, const int &size);


//�f�o�b�O�p�֐��@5*5�܂ł̑傫���̃}�b�v��\������
void printSegment(const int &width, const int &height, int *map);

/*�f�o�b�O�Z�b�g
int map[25] = {
	 1, -1, -1,  1, -1, 
	-1,  1, -1,  1,  1, 
	-1, -1, -1,  1, -1, 
	 1,  1, -1, -1, -1, 
	 1,  1,  1,  1,  1
};
int endNum;
segmentate(5, 5, map, 1, 2, endNum);
printSegment(5, 5, map);
while(1){
	int num;
	scanf("%d", &num);
	if(num > 5)break;
	printf("area : %d\n", getSegmentArea(25, num, map));
}
*/

/*
	int map[100];
	int endNum;
	segmentIkedaSearch(10, 10, map, 1, 2, endNum);
	printSegment(10, 10, map);
	while(1){
		for(int i = 0; i < 100; i++){
			if(rand() % 2)map[i] = -1;
			else map[i] = 1;
		}
		segmentate(10, 10, map, 1, 2, endNum);
		printSegment(10, 10, map);
		int num;
		scanf("%d", &num);
		if(num > 5)break;
	}*/