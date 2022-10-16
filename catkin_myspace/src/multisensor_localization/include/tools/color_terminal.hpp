/*
 * @Description: 终端彩色输出
 * @Function: 定义三个级别的彩色终端输出 蓝>黄>绿
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 * @Note
 */

#ifndef COLOR_TERMINAL_HPP_
#define COLOR_TERMINAL_HPP_

//c++
#include <string>

namespace multisensor_localization
{

/*终端彩色字体*/
#define fontColorReset "\033[0m"
#define fontColorBlack "\033[30m"
#define fontColorRed "\033[31m"
#define fontColorGreen "\033[32m"
#define fontColorYellow "\033[33m"
#define fontColorBlue "\033[34m"
#define fontColorMagenta "\033[35m"
#define fontColorCyan "\033[36m"
#define fontColorWhite "\033[37m"
#define fontColorBlackBold "\033[1m\033[30m"
#define fontColorRedBold "\033[1m\033[31m"
#define fontColorGreenBold "\033[1m\033[32m"
#define fontColorYellowBold "\033[1m\033[33m"
#define fontColorBlueBold "\033[1m\033[34m"
#define fontColorMagentaBold "\033[1m\033[35m"
#define fontColorCyanBold "\033[1m\033[36m"
#define fontColorWhiteBold "\033[1m\033[37m"

        class ColorTerminal
        {
        public:
                static void ColorNodeInfo(const std::string str);
                static void ColorFlowInfo(const std::string str);
                static void ColorConcreteInfo(const std::string str);
        };

} // namespace multisensor_localization

#endif