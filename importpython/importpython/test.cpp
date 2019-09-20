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

	Py_Initialize();//使用python之前，要调用Py_Initialize();这个函数进行初始化
	PyRun_SimpleString("import sys; sys.path.append('.')");
	PyObject * pModule = NULL;//声明变量
	PyObject * pFunc = NULL;// 声明变量
	PyObject *pargs =NULL;
	PyObject *reback =NULL;
	pModule =PyImport_ImportModule("test1");//这里是要调用的文件名
	pFunc= PyObject_GetAttrString(pModule, "func");//这里是要调用的函数名
	pargs=Py_BuildValue("(i)",a);
	PyEval_CallObject(pFunc, pargs);//调用函数

	pFunc= PyObject_GetAttrString(pModule, "GetFileList");//这里是要调用的函数名
	pargs=Py_BuildValue("(s)",filepath);
	reback=PyEval_CallObject(pFunc, pargs);//调用函数
	int size=PyTuple_Size(reback);

	for(int i=0;i<size;i++)//对元组中元素一个个读取
	{
		PyObject *pRet=PyTuple_GetItem(reback,i);
		PyArg_Parse(pRet,"s",&tmpstr);
		strcat(filelist,tmpstr);
		strcat(filelist," ");
	}
	//PyArg_ParseTuple(reback,"s",&filelist);
	printf("%s\n",filelist);
	system("pause");
	Py_Finalize();//调用Py_Finalize，这个根Py_Initialize相对应的。
}