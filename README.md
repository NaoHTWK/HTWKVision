# HTWKVision

## INSTALLATION

```
> mkdir build
> cd build
> cmake -DBUILD_DEMO_APP=ON ..
> make
> cp -r ../data .
> demo/demo --help
> demo/demo -i path/to/some/image.png -r
```

The Demo program will write a couple of PNGs with debug vision
data.

The shared library can be easily integrated into you build system.

It has no external dependencies.

A C++11 compatible compiler is needed to run the software.

## LICENSE 

Copyright (c) 2015 Nao-Team HTWK.  All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions 
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above 
   copyright notice, this list of conditions and the following 
   disclaimer in the documentation and/or other materials provided 
   with the distribution.

3. The end-user documentation included with the redistribution, if 
   any, must include the following acknowledgment:
   "This product includes software developed by Nao-Team HTWK
   ([htwk-robots.de](http://www.htwk-robots.de))."
   Alternately, this acknowledgment may appear in the software 
   itself, if and wherever such third-party acknowledgments 
   normally appear.

4. For each Nao-Team HTWK code release from which parts are used in
   a RoboCup competition, the usage shall be announced in the SPL 
   mailing list (currently robocup-nao@cc.gatech.edu) one month 
   before the first competition in which you are using it. The
   announcement shall name which parts of this code are used.

5. Bug fixes regarding existing code shall be sent back to
   Nao-Team HTWK via GitHub pull requests
   (https://github.com/NaoHTWK).

THIS SOFTWARE IS PROVIDED BY NAO-TEAM HTWK ``AS IS'' AND ANY
EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
NAO-TEAM HTWK NOR ITS MEMBERS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

