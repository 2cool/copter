#include "stdafx.h"
#include <windows.h>
#include <objidl.h>
#include <gdiplus.h>
#include <stdio.h>
#include "myButton.h"
#include "TButton.h"
#include "graph.h"
#include <windows.h>
#include <stdio.h>
#include <shellapi.h>
#include <stdio.h>
#include <io.h>
#include <fcntl.h>
#include <windows.h>
#include <iostream>
#include "Mpu.h"

void redraw_log();

using namespace std;
using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")

volatile bool redraw = false;
HWND hWndTEXT;
HWND hWndGPS;
HWND hWndGRAPH;

int mouse_pos = 0;
int xPos = 0;
int yPos = 0;

TButton *tb[ALL_ELEMENTS];
int tbuttons = 0;
myButton *mb[10];
int buttons = 0;
RECT winRect,gpsRect,textRect;
RectF graphField,horScroll,verScroll,gpsField;
int graph_off_x = 200;
int graph_off_y = 0;

double scrollX = 0, scrollY = 1;
bool scrollX_f=false, scrollY_f = false;
Graph *gr;
#define control_width 100
#define scroll_width 30








#include "gps.h"
#include "Text.h"




VOID OnPaint(HDC hdc)
{
	
	graphField.X = graph_off_x;
	graphField.Y = graph_off_y;
	graphField.Width = winRect.right- winRect.left - graph_off_x - control_width ;
	graphField.Height = winRect.bottom - winRect.top -graph_off_y - control_width ;

	horScroll.X = graph_off_x;
	horScroll.Width = graphField.Width;
	horScroll.Y = graphField.Y+graphField.Height + 20;
	horScroll.Height = scroll_width;

	verScroll.Y = graphField.Y;
	verScroll.Height = graphField.Height;
	verScroll.X = graphField.X + graphField.Width + 20;
	verScroll.Width = scroll_width;

	//------------------------------
	HDC memDC = CreateCompatibleDC(NULL);
	HDC scrDC = GetDC(0);
	HBITMAP bmp = CreateCompatibleBitmap(scrDC, winRect.right - winRect.left, winRect.bottom - winRect.top);
	ReleaseDC(0, scrDC);

	HBITMAP oldBitmap = (HBITMAP)SelectObject(memDC, bmp);
	DeleteObject(oldBitmap);




	Graphics graphics(memDC);


	//-------------------------------

	
	graphics.Clear(Color(255, 255, 255, 255));
	Pen      cpen(Color(255, 0, 0, 0));
	cpen.SetWidth(2);
	Pen      pen(Color(255, 0, 0, 0));

	graphics.DrawRectangle(&cpen, graphField);

	{
		graphics.DrawRectangle(&cpen, horScroll);
		int x = (int)(scrollX*horScroll.Width + horScroll.X);
		int y0 = horScroll.Y;
		int y1 = horScroll.Y + horScroll.Height;
		graphics.DrawLine(&cpen, x, y0, x, y1);
	}


	{
		
		graphics.DrawRectangle(&cpen, verScroll); 
		int y = (int)(scrollY*verScroll.Height + verScroll.Y);
		int x0 = verScroll.X;
		int x1 = verScroll.X + verScroll.Width;
		graphics.DrawLine(&cpen, x0, y, x1, y);
	}






	
	
	for (int i=0; i<buttons; i++)
		mb[i]->update(memDC);

	for (int i = 0; i < tbuttons; i++)
		tb[i]->update(memDC);

	gr->update(memDC, graphField,scrollY,scrollX);


	graphics.DrawLine(&pen, graphField.X, graphField.Y + graphField.Height / 2, graphField.X + graphField.Width, graphField.Y + graphField.Height / 2);

	if (BitBlt(hdc, 0, 0, winRect.right - winRect.left, winRect.bottom - winRect.top, memDC, 0, 0, SRCCOPY) == 0)
	{	// failed the blit
		DeleteDC(memDC);
		return ;
	}

	DeleteObject(bmp);
	DeleteDC(memDC);
	
}

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);




int n = 0;

int but1func1(int i) {
	n = i;
	return 0;
}

int but1func2(int i) {
	n = i;
	return 0;
}



// maximum mumber of lines the output console should have
static const WORD MAX_CONSOLE_LINES = 500;


void RedirectIOToConsole()
{
	int hConHandle;
	long lStdHandle;
	CONSOLE_SCREEN_BUFFER_INFO coninfo;
	FILE* fp;

	// allocate a console for this app
	AllocConsole();

	freopen("CONIN$", "r", stdin);
	freopen("CONOUT$", "w", stdout);
	freopen("CONOUT$", "w", stderr);
	
}


#include <string>

DWORD WINAPI myThread(LPVOID lpParameter)
{
	char buf[1000];
	cout << "write new variables there like\n\n";
	cout << "newQ,newR,dAngle,dangle_RC \n";
	cout << "-------------------------------------------\n\n";
	while (true) {
		cin >> buf;
		string s = string(buf);
		int eq=s.find_first_of("=");
		if (eq > 0) {
			string name = s.substr(0,eq);
			string val = s.substr(++eq);

			if (name.find("newR")==0) {
				Mpu.newR = stod(val);
				cout << name.c_str() << " = "<< Mpu.newR << endl;
				
			}
			else if (name.find("newQ") == 0) {
				Mpu.newQ = stod(val);
				cout << name.c_str() << " = " << Mpu.newQ << endl;
			}
			else if (name.find("dAngle") == 0) {
				Mpu.dAngle = stod(val);
				cout << name.c_str() << " = " << Mpu.dAngle << endl;
			}
			else if (name.find("dangle_RC") == 0) {
				Mpu.dangle_RC = stod(val);
				cout << name.c_str() << " = " << Mpu.dangle_RC << endl;
			}
			redraw = true;
			redraw_log();
		}

		
	}

	return 0;
}







INT WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, PSTR, INT iCmdShow)
{
	LPWSTR *szArgList;
	int argCount;

	szArgList = CommandLineToArgvW(GetCommandLine(), &argCount);
	if (szArgList == NULL)
	{
		MessageBox(NULL, L"Unable to parse command line", L"Error", MB_OK);
		return 10;
	}

	char c[1000] = { 0 };
	wcstombs(c, szArgList[1], wcslen(szArgList[1]));
	LocalFree(szArgList);

	gr= new Graph(c);

#define yButoonStep 14
	int yButton =-yButoonStep;
	int xButton = 10;




	//-----open conslole----------------------------------------------------------------------------------------------------------------
	RedirectIOToConsole();
	DWORD myThreadID;
	HANDLE myHandle = CreateThread(0, 0, myThread, 0, 0, 0);

	//---------------------------------------------------------------------------------------------------------------------







	
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"(f2)rotate", gr, ROTATE);
	yButton += 2;

	tb[tbuttons++] = new TButton(xButton, yButton+=yButoonStep, L"pitch", gr,PITCH);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"cp_pitch", gr, C_PITCH);

	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"roll", gr, ROLL);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"cp_roll", gr, C_ROLL);

	yButton += 2;
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"g_pitch", gr, GYRO_PITCH);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"g_roll", gr, GYRO_ROLL);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"g_yaw", gr, GYRO_YAW);
	yButton += 2;
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"F0  ", gr, F0);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"F1  ", gr, F1);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"F2  ", gr, F2);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"F3  ", gr, F3);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"THROTTLE", gr, THROTTLE);

	yButton += 2;
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"BAT ", gr, VOLTAGE);
	yButton += 2;


	yButton = -yButoonStep;
	xButton = 65;

	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"(f1)filter", gr, FILTER);
	yButton += 3;
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"accX", gr, ACCX);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"accY", gr, ACCY);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"accZ", gr, ACCZ);
	yButton += 5;

	
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"Yaw   ", gr, YAW);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"GPS_Yaw", gr, GPS_YAW);

	yButton += 5;
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"distX", gr, SX);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"distY", gr, SY);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"GdistX", gr, GSX);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"GdistY", gr, GSY);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"speedX", gr, SPEED_X);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"speedY", gr, SPEED_Y);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"GspeedX", gr, GSPEED_X);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"GspeedY", gr, GSPEED_Y);












	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"speedZ    ", gr, SPEED_Z);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"BAR_speedZ    ", gr, BAR_SPEED);

	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"  alt  ", gr, SZ);
    tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"bar_alt", gr, BAR_ALT);

	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"gps_alt", gr, GPS_ALT);


	xButton +=65;
	yButton = -yButoonStep;

	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"mi0 ", gr, MI0);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"mi1 ", gr, MI1);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"mi2 ", gr, MI2);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"mi3 ", gr, MI3);

	yButton += 10;
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"mag x ", gr, MX);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"mag y ", gr, MY);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"mag z ", gr, MZ);
	tb[tbuttons++] = new TButton(xButton, yButton += yButoonStep, L"Head", gr, M_HEAD);



	MSG                 msg;
	WNDCLASS            wndClass;
	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR           gdiplusToken;

	// Initialize GDI+.
	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	wndClass.style = CS_HREDRAW | CS_VREDRAW;
	wndClass.lpfnWndProc = WndProc;
	wndClass.cbClsExtra = 0;
	wndClass.cbWndExtra = 0;
	wndClass.hInstance = hInstance;
	wndClass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wndClass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndClass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wndClass.lpszMenuName = NULL;
	wndClass.lpszClassName = TEXT("graph_window");
	RegisterClass(&wndClass);

	wndClass.lpfnWndProc = WndProcGPS;
	wndClass.lpszClassName = TEXT("gps_window");
	RegisterClass(&wndClass);

	wndClass.lpfnWndProc = WndProcTEXT;
	wndClass.lpszClassName = TEXT("text_window");
	RegisterClass(&wndClass);





	hWndTEXT = CreateWindow(
		TEXT("text_window"),   // window class name
		TEXT("TEXT"),  // window caption
		WS_OVERLAPPEDWINDOW,      // window style
		700,            // initial x position
		400,            // initial y position
		400,            // initial x size
		700,            // initial y size
		NULL,                     // parent window handle
		NULL,                     // window menu handle
		hInstance,                // program instance handle
		NULL);

	ShowWindow(hWndTEXT, iCmdShow);
	UpdateWindow(hWndTEXT);

	hWndGRAPH = CreateWindow(
		TEXT("graph_window"),   // window class name
		TEXT("GRAPH"),  // window caption
		WS_OVERLAPPEDWINDOW,      // window style
		0,            // initial x position
		0,            // initial y position
		1920,            // initial x size
		400,            // initial y size
		NULL,                     // parent window handle
		NULL,                     // window menu handle
		hInstance,                // program instance handle
		NULL);       
	
	
	hWndGPS = CreateWindow(
		TEXT("gps_window"),   // window class name
		TEXT("GPS"),  // window caption
		WS_OVERLAPPEDWINDOW,      // window style
		0,            // initial x position
		400,            // initial y position
		700,            // initial x size
		700,            // initial y size
		NULL,                     // parent window handle
		NULL,                     // window menu handle
		hInstance,                // program instance handle
		NULL);
	
	// creation parameters

	ShowWindow(hWndGRAPH, iCmdShow);
	UpdateWindow(hWndGRAPH);


	ShowWindow(hWndGPS, iCmdShow);
	UpdateWindow(hWndGPS);

	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}

	GdiplusShutdown(gdiplusToken);
	

	return msg.wParam;
}  // WinMain






bool inRect(const int x, const int y, const RectF &r) {
	return (x >= r.X && x <= (r.X + r.Width) && y >= r.Y && y <= (r.Y + r.Height));
}



HWND hwnd;

void redraw_log() {
	if (redraw) {
		gr->readLog();
		RedrawWindow(hwnd, NULL, NULL, RDW_INVALIDATE);
		if (gps_hwnd != NULL)
			RedrawWindow(gps_hwnd, NULL, NULL, RDW_INVALIDATE);
		if (text_hwnd != NULL)
			RedrawWindow(text_hwnd, NULL, NULL, RDW_INVALIDATE);

		redraw = false;
	}
}




int old_filter = 3;

LRESULT CALLBACK WndProc(HWND hWnd, UINT message,
	WPARAM wParam, LPARAM lParam)
{
	HDC          hdc;
	PAINTSTRUCT  ps;
	
	hwnd = hWnd;
	switch (message)
	{


	case WM_MOUSEMOVE: {
		xPos = lParam & 0xffff;
		yPos = lParam >> 16;



		if (inRect(xPos, yPos, graphField)) {

			
			mouse_pos = xPos - graphField.X;

			///update_gps_marker = true;
		//	redraw = true;
			RedrawWindow(gps_hwnd, NULL, NULL, RDW_INVALIDATE);
			RedrawWindow(text_hwnd, NULL, NULL, RDW_INVALIDATE);
			
			
		}



		
		for (int i = 0; i < buttons; i++) {
			if (mb[i]->mouseMove(xPos, yPos))
				redraw = true;
				
		}
		for (int i = 0; i < tbuttons; i++) {
			//if (tb[i]->mouseMove(xPos, yPos))
			//	redraw = true;

		}
		if (redraw) {
			RedrawWindow(hWnd, NULL, NULL, RDW_INVALIDATE);
			RedrawWindow(text_hwnd, NULL, NULL, RDW_INVALIDATE);
			RedrawWindow(gps_hwnd, NULL, NULL, RDW_INVALIDATE);
		}
		return 0;
	}
	case WM_LBUTTONDOWN: {

		xPos = lParam&0xffff;
		yPos = lParam>>16;
		//bool redraw = false;

		if(inRect(xPos, yPos, horScroll)) {
			scrollX_f = true;
			scrollX = (double)(xPos-horScroll.X) / (double)horScroll.Width;
			redraw = true;
		}

		if (inRect(xPos, yPos, verScroll)) {
			scrollY_f = true;
			scrollY = (double)(yPos - verScroll.Y) / (double)verScroll.Height;
			redraw = true;
		}

		for (int i = 0; i < buttons; i++) {
			if (mb[i]->buttonDown(xPos, yPos))
				redraw = true;
		}
		for (int i = 0; i < tbuttons; i++) {
			if (tb[i]->buttonDown(xPos, yPos)) {
				//redraw = true;
			}
		}
		
		return 0;

	}

	case WM_LBUTTONUP: {

		xPos = lParam & 0xffff;
		yPos = lParam >> 16;
	
		for (int i = 0; i < buttons; i++) {
			if (mb[i]->buttonUp(xPos, yPos))
				redraw = true;;
		}
		for (int i = 0; i < tbuttons; i++) {
			if (tb[i]->buttonUp(xPos, yPos))
				redraw = true;;
		}
		if (redraw) {
			if (old_filter != gr->flags[FILTER]+(gr->flags[ROTATE]*2)) {
				old_filter = gr->flags[FILTER] + (gr->flags[ROTATE] * 2);

				
			}
			
		}

		redraw_log();

		return 0;

	}

	
	case WM_COMMAND: {
		char  text[1024];
		wchar_t wtext[1024];
		sprintf(text, "My variable is %d\n", lParam);
		mbstowcs(wtext, text, strlen(text) + 1);//Plus null
		
//		CheckDlgButton(hwndchBox, 1, BST_UNCHECKED);
		return 0;
	}
	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		GetWindowRect(hWnd, &winRect);

		OnPaint(hdc);
		EndPaint(hWnd, &ps);
		return 0;
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
} // WndProc