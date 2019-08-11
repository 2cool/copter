#pragma once

#include "stdafx.h"
#include <windows.h>
#include <objidl.h>
#include <gdiplus.h>
#include <stdio.h>
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")

class myButton
{
private:
	RectF r;
	int state;
	bool inRect(const int x, const int y);
	WCHAR *string;
	int(*do_s)(int);
public:
	myButton(WCHAR *string, int x, int y, int(*ff)(int));
	int update(HDC hdc);
	int buttonDown(int x, int y);
	int buttonUp(int x, int y);
	int mouseMove(int x, int y);
	~myButton();
};

