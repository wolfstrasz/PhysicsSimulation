#include "Window.h"

#include <iostream>
#include <cstring>
#include "key_binds.h"

Window::Window() :
	m_exitFlag(false), m_isFullscreen(false), m_width(800), m_height(600),
	m_targedFPS(30), m_fixedFPS(30), m_isVisible(false)
{
}

Window::Window(int width, int height) :
	m_exitFlag(false), m_isFullscreen(false), m_width(width), m_height(height),
	m_targedFPS(30), m_fixedFPS(30), m_isVisible(false)
{

}


void Window::Close() {
	m_exitFlag = true;
}

bool Window::GetExitFlag() {
	return m_exitFlag;
}

int Window::GetWidth() {
	return m_width;
}

int Window::GetHeight() {
	return m_height;
}

void Window::Resize(int width, int height) {
	m_width = width;
	m_height = height;
}

void Window::SetFullScreen(bool value) {
	m_isFullscreen = value;
}

bool Window::GetFullScreen() {
	return m_isFullscreen;
}

void Window::SetTargetFPS(int target) {
	m_targedFPS = target;
	if (m_targedFPS < 15) {
		m_targedFPS = 15;
	}
	if (m_targedFPS > 120) {
		m_targedFPS = 120;
	}
}

int Window::GetFixedFPS() {
	return m_fixedFPS;
}

void Window::MarkAsShown() {
	m_isVisible = true;
}

bool Window::WasWindowShown() {
	return m_isVisible;
}

int Window::GetTargetFPS() {
	return m_targedFPS;
}


int Window::KeyIndex(int keyCode) {
	switch (keyCode) {
	case KEY_NONE:              return 0;
	case KEY_PAUSE:             return 1;
	case KEY_SCROLL_LOCK:       return 2;
	case KEY_PRINT:             return 3;
	case KEY_SYSREQ:            return 4;
	case KEY_BREAK:             return 5;
	case KEY_ESCAPE:            return 6;
	case KEY_BACKSPACE:         return 7;
	case KEY_TAB:               return 8;
	case KEY_BACK_TAB:          return 9;
	case KEY_RETURN:            return 10;
	case KEY_CAPS_LOCK:         return 11;
	case KEY_SHIFT:             return 12;
	case KEY_CTRL:              return 13;
	case KEY_ALT:               return 14;
	case KEY_MENU:              return 15;
	case KEY_HYPER:             return 16;
	case KEY_INSERT:            return 17;
	case KEY_HOME:              return 18;
	case KEY_PG_UP:             return 19;
	case KEY_DELETE:            return 20;
	case KEY_END:               return 21;
	case KEY_PG_DOWN:           return 22;
	case KEY_LEFT_ARROW:        return 23;
	case KEY_RIGHT_ARROW:       return 24;
	case KEY_UP_ARROW:          return 25;
	case KEY_DOWN_ARROW:        return 26;
	case KEY_NUM_LOCK:          return 27;
	case KEY_KP_PLUS:           return 28;
	case KEY_KP_MINUS:          return 29;
	case KEY_KP_MULTIPLY:       return 30;
	case KEY_KP_DIVIDE:         return 31;
	case KEY_KP_ENTER:          return 32;
	case KEY_KP_HOME:           return 33;
	case KEY_KP_UP:             return 34;
	case KEY_KP_PG_UP:          return 35;
	case KEY_KP_LEFT:           return 36;
	case KEY_KP_FIVE:           return 37;
	case KEY_KP_RIGHT:          return 38;
	case KEY_KP_END:            return 39;
	case KEY_KP_DOWN:           return 40;
	case KEY_KP_PG_DOWN:        return 41;
	case KEY_KP_INSERT:         return 42;
	case KEY_KP_DELETE:         return 43;
	case KEY_SPACE:             return 56;
	case KEY_EXCLAM:            return 57;
	case KEY_QUOTE:             return 58;
	case KEY_NUMBER:            return 59;
	case KEY_DOLLAR:            return 60;
	case KEY_PERCENT:           return 61;
	case KEY_CIRCUMFLEX:        return 62;
	case KEY_AMPERSAND:         return 63;
	case KEY_APOSTROPHE:        return 64;
	case KEY_LEFT_PARENTHESIS:  return 65;
	case KEY_RIGHT_PARENTHESIS: return 66;
	case KEY_ASTERISK:          return 67;
	case KEY_PLUS:              return 68;
	case KEY_COMMA:             return 69;
	case KEY_MINUS:             return 70;
	case KEY_PERIOD:            return 71;
	case KEY_SLASH:             return 72;
	case KEY_ZERO:              return 73;
	case KEY_ONE:               return 74;
	case KEY_TWO:               return 75;
	case KEY_THREE:             return 76;
	case KEY_FOUR:              return 77;
	case KEY_FIVE:              return 78;
	case KEY_SIX:               return 79;
	case KEY_SEVEN:             return 80;
	case KEY_EIGHT:             return 81;
	case KEY_NINE:              return 82;
	case KEY_COLON:             return 83;
	case KEY_SEMICOLON:         return 84;
	case KEY_LESS_THAN:         return 85;
	case KEY_EQUAL:             return 86;
	case KEY_GREATER_THAN:      return 87;
	case KEY_QUESTION:          return 88;
	case KEY_AT:                return 89;
	case KEY_CAPITAL_A:         return 90;
	case KEY_CAPITAL_B:         return 91;
	case KEY_CAPITAL_C:         return 92;
	case KEY_CAPITAL_D:         return 93;
	case KEY_CAPITAL_E:         return 94;
	case KEY_CAPITAL_F:         return 95;
	case KEY_CAPITAL_G:         return 96;
	case KEY_CAPITAL_H:         return 97;
	case KEY_CAPITAL_I:         return 98;
	case KEY_CAPITAL_J:         return 99;
	case KEY_CAPITAL_K:         return 100;
	case KEY_CAPITAL_L:         return 101;
	case KEY_CAPITAL_M:         return 102;
	case KEY_CAPITAL_N:         return 103;
	case KEY_CAPITAL_O:         return 104;
	case KEY_CAPITAL_P:         return 105;
	case KEY_CAPITAL_Q:         return 106;
	case KEY_CAPITAL_R:         return 107;
	case KEY_CAPITAL_S:         return 108;
	case KEY_CAPITAL_T:         return 109;
	case KEY_CAPITAL_U:         return 110;
	case KEY_CAPITAL_V:         return 111;
	case KEY_CAPITAL_W:         return 112;
	case KEY_CAPITAL_X:         return 113;
	case KEY_CAPITAL_Y:         return 114;
	case KEY_CAPITAL_Z:         return 115;
	case KEY_LEFT_BRACKET:      return 116;
	case KEY_BACK_SLASH:        return 117;
	case KEY_RIGHT_BRACKET:     return 118;
	case KEY_UNDERSCORE:        return 119;
	case KEY_GRAVE:             return 120;
	case KEY_A:                 return 121;
	case KEY_B:                 return 122;
	case KEY_C:                 return 123;
	case KEY_D:                 return 124;
	case KEY_E:                 return 125;
	case KEY_F:                 return 126;
	case KEY_G:                 return 127;
	case KEY_H:                 return 128;
	case KEY_I:                 return 129;
	case KEY_J:                 return 130;
	case KEY_K:                 return 131;
	case KEY_L:                 return 132;
	case KEY_M:                 return 133;
	case KEY_N:                 return 134;
	case KEY_O:                 return 135;
	case KEY_P:                 return 136;
	case KEY_Q:                 return 137;
	case KEY_R:                 return 138;
	case KEY_S:                 return 139;
	case KEY_T:                 return 140;
	case KEY_U:                 return 141;
	case KEY_V:                 return 142;
	case KEY_W:                 return 143;
	case KEY_X:                 return 144;
	case KEY_Y:                 return 145;
	case KEY_Z:                 return 146;
	case KEY_LEFT_BRACE:        return 147;
	case KEY_BAR:               return 148;
	case KEY_RIGHT_BRACE:       return 149;
	case KEY_TILDE:             return 150;
	}
	return 0;
}