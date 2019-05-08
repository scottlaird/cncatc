# CncAtc

CncAtc is an Arduino program for connecting an AMB 1050 FME-W DI tool-changing
spindle to a GRBL CNC controller.  This is one of the cheapest ways to create a
working tool-changing CNC machine.

As it exists, this code is tightly tied to my machine, and is probably of
limited use to most people directly, but may be useful as an example.  It
doesn't currently make any attempt at being modular or runtime-configurable.  It
wasn't really written as an open source project, it was written to solve a
problem for me, and it shows evidence of multiple false starts and abandoned
approaches.

I'm releasing it because multiple people have asked me for a copy, and it's
reasonably readable as-is.  Unneeded features like current monitoring and air
pressure monitoring can be easily removed (or just ignored), and the core is
decent enough.  If there's interest, then I can work on making it more
generically useful over time.

### Disclaimer

This is not an officially supported Google product.
