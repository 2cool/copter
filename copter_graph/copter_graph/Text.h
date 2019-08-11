#pragma once

HWND text_hwnd = NULL;

LRESULT CALLBACK WndProcTEXT(HWND, UINT, WPARAM, LPARAM);
VOID OnPaintTEXT(HDC hdc) {
	RectF textField;
	textField.X = 0;
	textField.Y = 0;
	textField.Width = textRect.right - textRect.left;
	textField.Height = textRect.bottom - textRect.top;


	HDC memDC = CreateCompatibleDC(NULL);
	HDC scrDC = GetDC(0);
	HBITMAP bmp = CreateCompatibleBitmap(scrDC, textRect.right - textRect.left, textRect.bottom - textRect.top);
	ReleaseDC(0, scrDC);

	HBITMAP oldBitmap = (HBITMAP)SelectObject(memDC, bmp);
	DeleteObject(oldBitmap);









	double pos = scrollY*(double)(mouse_pos) / (double)horScroll.Width;

	pos = scrollX + (pos*(1 - scrollX));
	gr->drawText(memDC, textField, pos);


	if (BitBlt(hdc, 0, 0, textRect.right - textRect.left, textRect.bottom - textRect.top, memDC, 0, 0, SRCCOPY) == 0)
	{	// failed the blit
		DeleteDC(memDC);
		return;
	}

	DeleteObject(bmp);
	DeleteDC(memDC);

}
LRESULT CALLBACK WndProcTEXT(HWND hWnd, UINT message,
	WPARAM wParam, LPARAM lParam) {
	HDC          hdc;
	PAINTSTRUCT  ps;


	text_hwnd = hWnd;


	switch (message)
	{

	case WM_SETFOCUS:
		BringWindowToTop(hWndGPS);
		//BringWindowToTop(hWndGRAPH);
		return 0;
	

	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		GetWindowRect(hWnd, &textRect);

		OnPaintTEXT(hdc);
		EndPaint(hWnd, &ps);
		return 0;
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
}
