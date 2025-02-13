#ifndef RMCL_ROS_UTIL_TEXT_COLORS_H
#define RMCL_ROS_UTIL_TEXT_COLORS_H

// TEXT COLORS
#define TC_BLACK    "\033[1;30m"
#define TC_RED      "\033[1;31m"
#define TC_GREEN    "\033[1;32m"
#define TC_YELLOW   "\033[1;33m"
#define TC_BLUE     "\033[1;34m"
#define TC_MAGENTA  "\033[1;35m"
#define TC_CYAN     "\033[1;36m"
#define TC_WHITE    "\033[1;37m"

#define TC_END      "\033[0m"

#define TC_SENSOR   TC_YELLOW
#define TC_TOPIC    TC_CYAN
#define TC_FRAME    TC_MAGENTA
#define TC_MSG      TC_WHITE
#define TC_BACKENDS TC_BLUE


#endif // RMCL_ROS_UTIL_TEXT_COLORS_H