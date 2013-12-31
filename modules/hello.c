    /*
    hello.c: hello display module
     
    Copyright (C) 2013 Alessandro Pasotti
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program. If not, see .
    */
    #include <openchronos.h>
    /* drivers */
    #include "drivers/display.h"
    volatile uint8_t say_toggle = 0;
    // The callback
    static void say_hello(enum sys_message msg)
    {
    if(say_toggle){
    display_chars(0, LCD_SEG_L2_5_0, "HELLO", SEG_SET);
    } else {
    display_chars(0, LCD_SEG_L2_5_0, "WORLD", SEG_SET);
    }
    say_toggle = ! say_toggle;
    }
    /************************** menu callbacks ********************************/
    // Called on module activation
    static void hello_activate(void)
    {
    display_chars(0, LCD_SEG_L1_3_0, "HELL", SEG_SET);
    // register messagebus callback every second
    sys_messagebus_register(&say_hello, SYS_MSG_RTC_SECOND);
    }
    // Called on module deactivation
    static void hello_deactivate(void)
    {
    sys_messagebus_unregister(&say_hello);	
    }
    // Module intialization
    void mod_hello_init(void)
    {
    menu_add_entry("HELLO", NULL, NULL,
    NULL, NULL, NULL, NULL,
    &hello_activate, &hello_deactivate);
    }

