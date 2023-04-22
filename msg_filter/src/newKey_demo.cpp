/**
 * @file newKey_demo.cpp
 * @author semitia.top
 * @brief 按键输入demo，不阻塞版
 * @version 0.1
 * @date 2023-04-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

int main()
{
    char ch;
    int nread;
    struct termios old_settings, new_settings;

    //保存原来的终端属性
    tcgetattr(STDIN_FILENO, &old_settings);

    //设置新的终端属性
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO); //关闭规范模式和回显
    new_settings.c_cc[VTIME] = 0; //设置超时时间为0
    new_settings.c_cc[VMIN] = 0; //设置最小字符数为0

    //应用新的终端属性
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    while(1)
    {
        //尝试读取一个字符
        nread = read(STDIN_FILENO, &ch, 1);

        if(nread == 1) //如果读取成功
        {
            printf("You pressed %c\n", ch); //打印字符

            if(ch == 'q') //如果是q，退出循环
            {
                break;
            }
        }
        else //如果读取失败
        {
            printf("No input\n"); //打印提示信息
        }

        sleep(1); //暂停1秒
    }

    //恢复原来的终端属性
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);

    return 0;
}