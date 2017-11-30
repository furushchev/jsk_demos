jsk_2017_home_butler
===================

## TODO

- [ ] Speech
  - [x] synthesis
  - [ ] recognition
- [x] Reasoning stuff
- [ ] Object recognition
- [ ] Action interface using actionlib
- [ ] Action implementation for robots
  - [ ] PR2
  - [ ] Fetch
- [ ] Test codes
- [ ] Tutorials
- [ ] Logging

## Future work

- [ ] Integration with PDDL?
- [ ] Interactive semantic grounding

## Generate Grammar-to-Phoneme model

1. Install `g2p`

    - Go to http://www-i6.informatik.rwth-aachen.de/web/Software/g2p.html
      and download g2p source code.
    
    - Install python, numpy and swig.
    
        ```bash
        sudo apt install python-setuptools swig build-essential
        sudo easy_install pip
        sudo pip install -U pip setuptools
        sudo pip install numpy -U
        ```
    
    - Install g2p package
    
        ```bash
        cd /path/to/g2p
        python setup.py build_ext
        sudo python setup.py install
        ```

2. Download dictionary

    - Go to cmudict https://github.com/cmusphinx/cmudict
      and download dictionary

3. Train models

    ```bash
    $ g2p.py --train cmudict.dict --devel 5% --write-model model-1
    $ g2p.py --model model-1 --ramp-up --train cmudict.dict --devel 5% --write-model model-2
    ...  # repeat several times
    ```

4. Transcribe new words

    ```bash
    $ cat <<EOF >> words.txt
    cat
    smartphone
    euslisp
    EOF
    $ g2p.py --model model-3 --apply words.txt
    cat         K AH0 T
    smartphone  S M AH0 R T P N
    euslisp     S L IH0 S P
    stack usage:  2
    ```


## Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>
