1.再补一个很值得开的保护：

在 FreeRTOSConfig.h 里加 configCHECK_FOR_STACK_OVERFLOW 2
实现 vApplicationStackOverflowHook()
这样下次会更早告诉你“栈溢出”，而不是等到删任务时才炸

---

2.bsp_gpio bsp_board的初始化和回调注册没有线程安全