#ifndef COLOR_TERMINAL_HPP_
#define COLOR_TERMINAL_HPP_

#include <string>

namespace glins
{

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
                static void NodeInfo(const std::string str);
                static void FlowInfo(const std::string str);
    
    };//ColorTerminal

} // namespace glins

#endif