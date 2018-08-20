# pick-place
Pick and place robot: pick up payload from location x and drop accurately and quickly to position y. 

Mechanical design of a truss cantilever to act as a crane arm. A DC motor is placed at the base to rotate the crane by 360 degrees. 

An electromagnet is placed at the tip of the crane, and is energized to pick up a metal payload. The truss then moves to the desired angle and drops the payload by de-energizing the electromagnet. 

The DC motor is driven by an H-Bridge. The circuit is designed and laid out onto a small PCB. 

The H-bridge signals and electromagnet are controlled using a Tiva C microcontroller. It takes in quadrature encoder inputs, and outputs the speed & direction for the DC motor, in addition to an enable signal for the electromagnet. The final position for the crane is controlled in a feedback loop using a PID controller with the desired 'place' location being the reference input.

![Pick and Place Truss](https://media.giphy.com/media/5PhDNnBHIBmpxO9xGE/giphy.gif)
