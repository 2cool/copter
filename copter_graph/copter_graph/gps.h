#pragma once



HWND gps_hwnd = NULL;

LRESULT CALLBACK WndProcGPS(HWND, UINT, WPARAM, LPARAM);
VOID OnPaintGPS(HDC hdc) {
	
	gpsField.X = 0;
	gpsField.Y = 0;
	gpsField.Width = gpsRect.right - gpsRect.left;
	gpsField.Height = gpsRect.bottom - gpsRect.top;


	HDC memDC = CreateCompatibleDC(NULL);
	HDC scrDC = GetDC(0);
	HBITMAP bmp = CreateCompatibleBitmap(scrDC, gpsRect.right - gpsRect.left, gpsRect.bottom - gpsRect.top);
	ReleaseDC(0, scrDC);

	HBITMAP oldBitmap = (HBITMAP)SelectObject(memDC, bmp);
	DeleteObject(oldBitmap);

	gr->updateGPS(memDC, gpsField, scrollY, scrollX);

	double pos = scrollY*(double)(mouse_pos) / (double)horScroll.Width;

	pos = scrollX + (pos*(1 - scrollX));
	gr->drawGPSmarkder(memDC, gpsField, pos);

	if (BitBlt(hdc, 0, 0, gpsRect.right - gpsRect.left, gpsRect.bottom - gpsRect.top, memDC, 0, 0, SRCCOPY) == 0)
	{	// failed the blit
		DeleteDC(memDC);
		return;
	}

	DeleteObject(bmp);
	DeleteDC(memDC);
	

}
LRESULT CALLBACK WndProcGPS(HWND hWnd, UINT message,
	WPARAM wParam, LPARAM lParam) {
	HDC          hdc;
	PAINTSTRUCT  ps;


	gps_hwnd = hWnd;


	switch (message)
	{
	case WM_SETFOCUS:
		//BringWindowToTop(hWndGPS);
		BringWindowToTop(hWndGRAPH);
		return 0;
	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		GetWindowRect(hWnd, &gpsRect);

		OnPaintGPS(hdc);
		EndPaint(hWnd, &ps);
		return 0;
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
}