.string "Hello, world\n" message
main:
    load A, message      // a is equal to pointer to first letter
    .loop
        load G, @0+A     // load g with the char
        cmp G, 0         // if compare g and 0
        eq exit          // if g == 0, jump to exit
        load @65276, G   // load char into 0xFEFC
        load @65277, 1   // load 1 into 0xFEFD; invokes syscall, prints
        addi A, 1        // a++
        jmp .loop        // goto loop
exit:                
    jmp exit             // loop forever
.reset main
