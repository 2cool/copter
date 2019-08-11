#pragma once


#include <windows.h>
#include <objidl.h>
#include <gdiplus.h>
#include <stdio.h>


using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")



#include "graph.h"


class TButton
{
private:
	RectF r;
	WCHAR *string;
	bool inRect(const int x, const int y);
	bool  state = false;
	Graph *gr;
	int gr_index;
public:
	int update(HDC hdc);
	TButton(const int x, const int y, WCHAR *str, Graph *gr,const int gr_index);
	~TButton();
	int buttonDown(int x, int y);
	int buttonUp(int x, int y);
	int mouseMove(int x, int y);
};

