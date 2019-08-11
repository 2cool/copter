#include "stdafx.h"
#include "myButton.h"



myButton::myButton(WCHAR *str, int x, int y, int(*ff)(int))
{
	string = str;
	r.X = x;
	r.Y = y;
	r.Width = 60;
	r.Height = 20;
	state = 0;
	do_s = ff;

}

int myButton::update(HDC hdc){
	Pen      pen(Color(255, (state) ? 255 : 0, 0, 0));
	Graphics g(hdc);
	g.DrawRectangle(&pen, r);

	//WCHAR string[] = L"Text";

	// Initialize arguments.
	Font myFont(L"Arial", 14);

	StringFormat format;
	format.SetAlignment(StringAlignmentCenter);
	SolidBrush blackBrush(Color(255, (state)?255:0, 0, 0));

	// Draw string.
	g.DrawString(
		string,
		sizeof(string),
		&myFont,
		r,
		&format,
		&blackBrush);
	

	
	return 0;
}

myButton::~myButton()
{
}
bool myButton::inRect(const int x, const int y) {
	return (x >= r.X && x <= (r.X + r.Width) && y >= r.Y && y <= (r.Y + r.Height));
}
int myButton::buttonDown(int x, int y) {
	if ( inRect(x,y)){
		state = 1;
		return true;
	}
	
	return false;
		
}
int myButton::buttonUp(int x, int y) {
	if (state == 1 && inRect(x, y)) {
		do_s(0);
		state = 0;
		return true;
	}
	state = 0;
	return false;
}

int myButton::mouseMove(int x, int y) {

	if (inRect(x, y) == false && state == 1) {
		state = 0;
		return 1;
	}

	return 0;
}