#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from textblob import TextBlob
from textblob.exceptions import MissingCorpusError
try:
    t = TextBlob("I found a table")
    t.tags
except MissingCorpusError:
    from textblob.download_corpora import download_all
    download_all()


class CommandPreprocessor(object):
    def __init__(self,
                 definitize=True,
                 lemmatize=True):
        self.enable_definitize = definitize
        self.enable_lemmatize = lemmatize

    def process(self, sentence):
        assert isinstance(sentence, str)
        sentence = sentence.lower()
        if self.enable_definitize:
            sentence = self.definitize(sentence)
        if self.enable_lemmatize:
            sentence = self.lemmatize(sentence)

        return sentence

    def _concat(self, result):
        words = [w for w, t in result]
        sentence = ""
        for w in words:
            if w.startswith("'"):
                sentence += w
            else:
                sentence += " " + w
        return sentence.strip()

    def definitize(self, sentence):
        words = TextBlob(sentence).tags
        result = []

        for i, (w, t) in enumerate(words):
            if w.lower() == 'a' and t == 'DT':
                if i+1 >= len(words) or words[i+1][0].lower() == 'person':
                    result.append((w, t))
                else:
                    result.append(('the', 'DT'))
            else:
                result.append((w, t))

        return self._concat(result)

    def lemmatize(self, sentence):
        words = TextBlob(sentence).tags
        result = []
        for w, t in words:
            if t in ['VBN'] and w.lemmatize("v"):
                result.append((w.lemmatize("v"), 'VB'))
            else:
                result.append((w, t))

        return self._concat(result)



if __name__ == '__main__':

    # q = "Delivered a manju to Samantha at the bed"
    # q = "Navigate to the bathroom, find someone, and say your team's name"
    q = "Navigate to the office, locate Alex, and tell something about yourself"
    q = "find a person in the corridor and please choose A"

    p = CommandPreprocessor()

    r = p.process(q)

    print r
