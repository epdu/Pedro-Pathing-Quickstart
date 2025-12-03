package org.firstinspires.ftc.teamcode.pedroPathing;

public class ButtonHandler {
    private boolean isPressed = false;
    private long pressStartTime = 0;
    private boolean isLongPressHandled = false;
    private int clickCount = 0;
    private long lastClickTime = 0;

    private static final long LONG_PRESS_THRESHOLD = 500; // Long press time threshold (milliseconds)
    private static final long DOUBLE_CLICK_INTERVAL = 300;// Long press time threshold (milliseconds)

    public void update(boolean isButtonPressed) {
        long currentTime = System.currentTimeMillis();

        if (isButtonPressed) {
            if (!isPressed) { // The button was just pressed
                isPressed = true;
                pressStartTime = currentTime;
                clickCount++;
            } else if (currentTime - pressStartTime >= LONG_PRESS_THRESHOLD && !isLongPressHandled) {
                isLongPressHandled = true; // Mark that the long press has been handled
            }
        } else { // 按键释放
            if (isPressed) {
                if (isLongPressHandled) {
                    // If it's a long press, reset the long press handling flag after the button is released
                    isLongPressHandled = false;
                } else {
                    // 短按逻辑
                    if (currentTime - lastClickTime > DOUBLE_CLICK_INTERVAL) {
                        clickCount = 1; // 如果超过双击间隔，重置点击计数
                    }
                }
                lastClickTime = currentTime;
            }
            isPressed = false; // 按键状态重置
        }
    }

    public boolean isShortPress() {
        return !isPressed && clickCount == 1 && !isLongPressHandled;
    }

    public boolean isLongPress() {
        return isPressed && isLongPressHandled;
    }

    public boolean isDoubleClick() {
        return !isPressed && clickCount == 2;
    }

    public void reset() {
        clickCount = 0;
        isPressed= false;
        isLongPressHandled = false;
    }
}