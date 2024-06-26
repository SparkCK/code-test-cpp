#include <iostream>
#include <string>
#include <ncurses.h> // 用于获取键盘输入

int main()
{
    initscr(); // 初始化ncurses
    cbreak(); // 禁用行缓冲
    noecho(); // 不回显输入字符
    keypad(stdscr, TRUE); // 启用特殊键（如箭头键）

    bool paused = false;
    for (int i = 0; i < 10; i++)
    {
        // 检测键盘输入
        if (wgetch(stdscr) == ' ')
        {
            paused = !paused; // 切换暂停状态
        }

        // 如果未暂停，则打印当前值
        if (!paused)
        {
            printw("i = %d\n", i);
            refresh(); // 刷新屏幕
        }
    }

    endwin(); // 结束ncurses
    return 0;
}
