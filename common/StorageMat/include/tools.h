
#ifndef TOOLS_H
#define TOOLS_H

#include <string>

/**
 * condition为false时抛出错误，错误信息为error_message
 */
#define ASSERT(condition,error_message) \
    if (!(condition)){\
        error(__FILE__, __func__, __LINE__,error_message); \
    }
/**
 * condition为false时输出警告，错误信息为warning_message
 */
#define WARNING(condition,warning_message)\
    if (!(condition)){\
        warning(__FILE__, __func__, __LINE__,error_message); \
    }

void error(const std::string &filePath, const std::string &function,
           int line, const std::string &info);

void warning(const std::string &filePath, const std::string &function,
                    int line, const std::string &info);

/**
 * condition为true时不做任何动作;
 * condition为false时输出false_message,并返回false_value.
 */
#define IF(condition,false_message,false_value) \
    if (!(condition)){\
        std::cerr<<"Line:"<<__LINE__<<">>"<<(false_message)<<std::endl; \
        return (false_value); \
    }
#endif //TOOLS_H
