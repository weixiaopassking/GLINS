/*
 * @Description: 终端彩色输出
 * @Function: 定义三个级别的彩色终端输出 蓝>黄>绿
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 * @Note
 */

// directly relevant
#include "../../include/tools/color_terminal.hpp"
// c++
#include <string>
#include <iostream>

namespace multisensor_localization
{

    /**
     * @brief 彩色终端输出--节点级别
     * @note 蓝色
     * @todo
     **/
    void ColorTerminal::ColorNodeInfo(const std::string str)
    {
        std::cout << std::endl
                  << fontColorBlueBold << str << fontColorReset << std::endl
                  << std::endl;
    }

    /**
     * @brief 彩色终端输出--任务管理级别
     * @note 黄色
     * @todo
     **/
    void ColorTerminal::ColorFlowInfo(const std::string str)
    {
        std::cout << std::endl
                  << fontColorYellowBold << str << fontColorReset << std::endl
                  << std::endl;
    }

    /**
     * @brief 彩色终端输出--算法实现级别
     * @note 绿色
     * @todo
     **/
    void ColorTerminal::ColorConcreteInfo(const std::string str)
    {
        std::cout << std::endl
                  << fontColorGreenBold << str << fontColorReset << std::endl
                  << std::endl;
        ;
    }


        /**
     * @brief 彩色终端输出--算法实现级别
     * @note 绿色
     * @todo
     **/
    void ColorTerminal::ColorConcreteDebug(const std::string str)
    {
        std::cout << std::endl
                  << fontColorWhiteBold << str << fontColorReset << std::endl
                  << std::endl;
        ;
    }

} // namespace multisensor_localization
