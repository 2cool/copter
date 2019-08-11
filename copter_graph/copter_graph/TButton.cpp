#include "stdafx.h"
#include "TButton.h"


TButton::TButton(const int x, const int y, WCHAR *str, Graph *gr, const int gr_index)
{
	this->gr = gr;
	r.X = x;
	r.Y = y;
	r.Width = 50;
	r.Height = 12;

	string = str;
	this->gr_index = gr_index;

}
int TButton::update(HDC hdc) {
	Pen      pen(Color(255,  0, 0, 0));
	Graphics g(hdc);
	
	




	//WCHAR string[] = L"Text";

	// Initialize arguments.
	Font myFont(L"Arial", 8);

	StringFormat format;
	format.SetAlignment(StringAlignmentCenter);
	SolidBrush coloredBrush(gr->color[gr_index]);
	SolidBrush blackBrush(Color(255,  0, 0, 0));
	
	// Draw string.

	if (gr->flags[gr_index])
		g.FillRectangle(&coloredBrush, r);
	g.DrawString(
		string,
		sizeof(string),
		&myFont,
		r,
		&format,
		&blackBrush);

	g.DrawRectangle(&pen, r);

	return 0;
}

TButton::~TButton()
{
}
bool TButton::inRect(const int x, const int y) {
	return (x >= r.X && x <= (r.X + r.Width) && y >= r.Y && y <= (r.Y + r.Height));
}
int TButton::buttonDown(int x, int y) {
	if (inRect(x, y)) {
		state = 1;
		return true;
	}

	return false;

}
int TButton::buttonUp(int x, int y) {
	if (state == 1 && inRect(x, y)) {
		gr->flags[gr_index] ^=true;
		state = 0;
		return true;
	}
	state = 0;
	return false;
}

int TButton::mouseMove(int x, int y) {

	if (inRect(x, y) == false && state == 1) {
		state = 0;
		return 1;
	}

	return 0;
}