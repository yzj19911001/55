#include "stdafx.h"
#include <Python.h>
#include<iostream>

#include <windows.h>
using namespace std;
char Path[100]={0};

void main()
{
	char filepath[50]={0};
	char filelist[100]={0};
	char *tmpstr;
	sprintf(filepath,"C:\\Users\\wanji\\Desktop\\test4");
	int a=3;

	Py_Initialize();//ʹ��python֮ǰ��Ҫ����Py_Initialize();����������г�ʼ��
	PyRun_SimpleString("import sys; sys.path.append('.')");
	PyObject * pModule = NULL;//��������
	PyObject * pFunc = NULL;// ��������
	PyObject *pargs =NULL;
	PyObject *reback =NULL;
	pModule =PyImport_ImportModule("test1");//������Ҫ���õ��ļ���
	pFunc= PyObject_GetAttrString(pModule, "func");//������Ҫ���õĺ�����
	pargs=Py_BuildValue("(i)",a);
	PyEval_CallObject(pFunc, pargs);//���ú���

	pFunc= PyObject_GetAttrString(pModule, "GetFileList");//������Ҫ���õĺ�����
	pargs=Py_BuildValue("(s)",filepath);
	reback=PyEval_CallObject(pFunc, pargs);//���ú���
	int size=PyTuple_Size(reback);

	for(int i=0;i<size;i++)//��Ԫ����Ԫ��һ������ȡ
	{
		PyObject *pRet=PyTuple_GetItem(reback,i);
		PyArg_Parse(pRet,"s",&tmpstr);
		strcat(filelist,tmpstr);
		strcat(filelist," ");
	}
	//PyArg_ParseTuple(reback,"s",&filelist);
	printf("%s\n",filelist);
	system("pause");
	Py_Finalize();//����Py_Finalize�������Py_Initialize���Ӧ�ġ�
}