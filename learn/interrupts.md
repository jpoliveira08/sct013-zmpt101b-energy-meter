Count events independently from the loop

Interrupts: They interrupt the loop wherever it is, save the relevant data in a particular place (stack or heap) and executes a different function, when the function is finish, reloads all data from stack or heap and give the controller back to the loop.

If the function is short, the skect doesn't recognize, it runs like in parallel.

Not using Serial.print() or delay (time consuming comands)

First we have to attach the interrupt

attachInterrupt(
    INPUTPIN, the pint number
    isr, the isr function to call
    RISING // Event (Rising or Falling edges)
);

The isr function is called every time the pings go high automatically
void IRAM_ATTR isr() {
    count++;
}

We can use delay without any problem in the loop

