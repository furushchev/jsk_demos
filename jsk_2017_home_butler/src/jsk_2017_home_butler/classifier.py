#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospkg
import os
import sys
import numpy as np
import pickle
from sklearn.feature_extraction.text import TfidfVectorizer
import rospy


PKG_PATH = rospkg.RosPack().get_path("jsk_2017_home_butler")


class Phonemizer(object):
    def phonemize(self, sentence):
        raise NotImplementedError()


class PronouncingPhonemizer(Phonemizer):
    def __init__(self, debug=True):
        super(PronouncingPhonemizer, self).__init__()

        self.debug = debug

        self.load_modules()

        dict_path = rospy.get_param("~phoneme_dict_path", None)
        if dict_path is None:
            dict_path = os.path.join(PKG_PATH, "data", "phonemes.txt")

        self.dict = self.load_dicts(dict_path)
        rospy.loginfo("loaded %d entities" % len(self.dict))

    def load_modules(self):
        global pronouncing
        global splitter
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

    def phonemize_word(self, word):
        phoneme = pronouncing.phones_for_word(word)
        if phoneme:
            return phoneme[0]
        elif word in self.additional_dict.keys():
            return self.additional_dict[word]
        else:
            raise RuntimeError("No phoneme found: %s. Please add to additional dict." % word)

    def split_sentence(self, sentence):
        sentence = sentence.strip()
        if sentence.find(' ') != -1:
            return sentence.split()
        else:
            ss = splitter.split(sentence)
            return ss if ss else [sentence]

    def phonemize(self, sentence):
        phonemes = []
        try:
            phonemes = [self.phonemize_word(sentence)]
        except:
            if phonemes:
                return ' '.join(phonemes)
            for s in self.split_sentence(sentence):
                phoneme = self.phonemize_word(s)
                if not phoneme:
                    phoneme = ' '.join([self.phonemize(ss) for ss in self.split_sentence(s)])
                phonemes.append(phoneme)
            if self.debug:
                print sentence, "->", ' '.join(phonemes)
        return ' '.join(phonemes)


class SequiturPhonemizer(Phonemizer):
    def __init__(self, debug=True):
        super(SequiturPhonemizer, self).__init__()
        self.load_modules()

        model_path = rospy.get_param("~phoneme_model_file", None)
        if model_path is None:
            model_path = os.path.join(PKG_PATH, "data", "phonemes.model")
        assert os.path.exists(model_path), "Phonemizer model not found at %s" % model_path

        print "Loading phonemizer model..."
        with open(model_path, "rb") as f:
            model = pickle.load(f)
        self.translator = sequitur.Translator(model)
        del model

    def load_modules(self):
        global sequitur
        try:
            import sequitur
        except:
            print >> sys.stderr, "'sequitur' library not found. Please install sequitur G2P first."
            exit(1)

    def _phonemize_word(self, word):
        return self.translator(word.strip())

    def phonemize(self, sentence):
        result = list()
        for word in sentence.strip().split():
            result += self._phonemize_word(word)
        return ' '.join(result)


class WordClassifier(object):
    def __init__(self,
                 classifier='randomforest',
                 phonemizer='sequitur',
                 use_phonemes=False,
                 debug=False):

        self.debug = debug

        self.vectorizer = TfidfVectorizer(ngram_range=(1, 2),
                                          strip_accents='ascii',
                                          sublinear_tf=True,
                                          max_df=0.5,
                                          stop_words='english')
        self.classifier_name = classifier
        self.clf = None

        self.commands = []
        self.phonemes = []

        self.use_phonemes = use_phonemes
        self.phonemizer_name = phonemizer
        self.phonemizer = None

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

    def _init_phonemizer(self, name):
        if name == 'pronouncing':
            self.phonemizer = PronouncingPhonemizer(debug=self.debug)
        elif name == 'sequitur':
            self.phonemizer = SequiturPhonemizer(debug=self.debug)
        else:
            raise RuntimeError("Phonemizer '%s' not found" % name)

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
            if self.phonemizer is None:
                self._init_phonemizer(self.phonemizer_name)
            self.phonemes = [self.phonemizer.phonemize(c) for c in commands]
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
            input_data = self.phonemizer.phonemize(command)
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
