#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospkg
import os
import sys
import numpy as np
from sklearn.feature_extraction.text import TfidfVectorizer
import rospy
try:
    import pronouncing
except:
    print >> sys.stderr, "'pronouncing' library not found. Please run 'sudo pip install pronouncing'"
    exit(1)
try:
    import splitter
except:
    print >> sys.stderr, "'compound-word-splitter' library not found. Please run 'sudo pip install compound-word-splitter pyenchant'"
    exit(1)

PKG_PATH = rospkg.RosPack().get_path("jsk_2017_home_butler")

class WordClassifier(object):
    def __init__(self, classifier='randomforest',
                 use_phonemes=False, additional_dict_path=None,
                 debug=False):
        if additional_dict_path is None:
            default_path = os.path.join(PKG_PATH, "data", "phonemes.txt")
            if os.path.exists(default_path):
                additional_dict_path = default_path

        self.vectorizer = TfidfVectorizer(ngram_range=(1, 2),
                                          strip_accents='ascii',
                                          sublinear_tf=True,
                                          max_df=0.5,
                                          stop_words='english')
        self.classifier_name = classifier
        self.clf = None

        # load additional dictionaries
        self.additional_dict = {}
        if additional_dict_path is not None:
            self.additional_dict = self.load_dicts(additional_dict_path)

        self.commands = []
        self.phonemes = []

        self.use_phonemes = use_phonemes
        self.debug = debug

    def _init_classifier(self, classifier, n_class):
        if classifier == 'randomforest':
            from sklearn.ensemble import RandomForestClassifier
            self.clf = RandomForestClassifier(
                n_estimators=100, n_jobs=-1,
            )
        elif classifier == 'ridge':
            from sklearn.linear_model import RidgeClassifier
            self.clf = RidgeClassifier(tol=1e-2, solver='lsqr')
        elif classifier == 'perceptron':
            from sklearn.linear_model import Perceptron
            self.clf = Perceptron(n_iter=50)
        elif classifier == 'kneighbors':
            from sklearn.neighbors import KNeighborsClassifier
            self.clf = KNeighborsClassifier(n_neighbors=n_class)
        elif classifier == 'passive_aggressive':
            from sklearn.linear_model import PassiveAggressiveClassifier
            self.clf = PassiveAggressiveClassifier(n_iter=n_class)
        else:
            raise RuntimeError("classifier '%s' not found" % classifier)

    def load_dicts(self, path):
        dic = {}
        if not os.path.exists(path):
            raise IOError("dict %s not found" % path)
        with open(path, 'r') as f:
            for line in f.readlines():
                line = line.strip()
                if not line: continue
                elif line[0] == '#': continue
                if len(line.split(':')) != 2:
                    raise ValueError("Failed to parse dict: %s" % line)
                word, phoneme = line.split(':')
                dic[word.strip()] = phoneme.strip()
        rospy.loginfo("%d additional dictionary loaded" % len(dic))
        return dic

    def _get_phoneme(self, word):
        phoneme = pronouncing.phones_for_word(word)
        if phoneme:
            return phoneme[0]
        elif word in self.additional_dict.keys():
            return self.additional_dict[word]
        else:
            raise RuntimeError("No phoneme found: %s. Please add to additional dict." % word)

    def _split_sentence(self, sentence):
        sentence = sentence.strip()
        if sentence.find(' ') != -1:
            return sentence.split()
        else:
            ss = splitter.split(sentence)
            return ss if ss else [sentence]

    def _get_phonemes(self, sentence):
        phonemes = []
        try:
            phonemes = [self._get_phoneme(sentence)]
        except:
            if phonemes:
                return ' '.join(phonemes)
            for s in self._split_sentence(sentence):
                phoneme = self._get_phoneme(s)
                if not phoneme:
                    phoneme = ' '.join([self._get_phonemes(ss) for ss in self._split_sentence(s)])
                phonemes.append(phoneme)
            if self.debug:
                print sentence, "->", ' '.join(phonemes)
        return ' '.join(phonemes)

    def _sanitize_command(self, command):
        command = command.replace('-', ' ')
        command = command.replace('_', ' ')
        command = command.lower()
        return command

    def fit(self, commands):
        assert isinstance(commands, list)
        commands = [self._sanitize_command(c) for c in commands]
        self.commands = commands

        if self.clf is None:
            self._init_classifier(self.classifier_name, len(self.commands))

        if self.use_phonemes:
            self.phonemes = [self._get_phonemes(c) for c in commands]
            input_data = self.phonemes
        else:
            input_data = commands
        train_x = self.vectorizer.fit_transform(input_data)
        train_y = np.arange(len(commands))
        return self.clf.fit(train_x.toarray(), train_y)

    def infer(self, command, with_proba=True):
        if self._sanitize_command(command) in self.commands:
            if with_proba:
                return command, 1.0
            else:
                return command
        command = self._sanitize_command(command)
        if self.use_phonemes:
            input_data = self._get_phonemes(command)
        else:
            input_data = command
        x = self.vectorizer.transform([input_data])
        proba = self.clf.predict_proba(x.toarray())[0]

        if self.debug:
            n = min(proba.shape[0], 10)
            topn = proba.argsort()[-n:][::-1]
            for i, y in enumerate(topn):
                fmt = "#%d: %s (%.2f)" % (i, self.commands[y], proba[y])
                if self.use_phonemes:
                    fmt += " (%s)" % self.phonemes[y]
                print fmt

        y = np.nanargmax(proba)
        if with_proba:
            return self.commands[y], proba[y]
        else:
            return self.commands[y]


if __name__ == '__main__':
    from jsk_2017_home_butler.resource_loader import ResourceLoader

    def location_test():
        loader = ResourceLoader()

        locations = [l["location"] for l in loader.locations]
        locations += [l["room"] for l in loader.locations]
        locations = list(set(locations))
        print locations
        i = WordClassifier(use_phonemes=True, debug=True)

        print "--- learning ---"
        i.fit(locations)

        print
        print "--- inferring ---"
        word = "diving room"
        result = i.infer(word)
        print "infer:", word, "->", result
        word = "livingtable"
        result = i.infer(word)
        print "infer:", word, "->", result
        word = "center pebble"
        result = i.infer(word)
        print "infer:", word, "->", result
        word = "think"
        result = i.infer(word)
        print "infer:", word, "->", result
        word = "Thai shelf"
        result = i.infer(word)
        print "infer:", word, "->", result
        word = "corridor"
        result = i.infer(word)
        print "infer:", word, "->", result

    def object_test():
        loader = ResourceLoader()

        objects = [o["name"] for o in loader.objects]
        objects += [o["category"] for o in loader.objects]
        objects = list(set(objects))
        i = WordClassifier(use_phonemes=True, debug=True)
        print "--- learning ---"
        i.fit(objects)

        print
        print "--- inferring ---"
        for word in ["manju", "green T", "coke"]:
            result = i.infer(word)
            print "infer:", word, "->", result

    print "============== Location ==============="
    location_test()

    print
    print "============== Object ================"
    object_test()
