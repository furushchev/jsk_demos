#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import copy
import rospy
from jsk_2017_home_butler.action_loader import ActionLoader
from jsk_2017_home_butler.resource_loader import ResourceLoader
from jsk_2017_home_butler.classifier import WordClassifier
from jsk_2017_home_butler.unknown_object_database import UnknownObjectDatabase


class CommandInterpolateError(Exception):
    def __init__(self, message, command, valid_commands, all_commands, state):
        super(CommandInterpolateError, self).__init__(message)
        self.valid_commands = valid_commands
        self.all_commands = all_commands
        self.command = command
        self.state = state

    def __repr__(self):
        fmt = "(%s " % self.command.name
        fmt += ' '.join(['%s:%s' % (k, v) for k, v in self.command.arguments.items()])
        fmt += ") at %s" % self.state
        return fmt


class UnknownSymbolError(CommandInterpolateError):
    pass


class CommandInterpolator(object):
    def __init__(self, align=True):
        self.resource = ResourceLoader()

        self.initial_state = {
            'location': 'operatorfront',
            "near_person": None,
            "near_object": None,
            "on_hand": None,
        }

        self.align = align
        if self.align:
            objects = [o["name"] for o in self.resource.objects]
            objects += [o["category"] for o in self.resource.objects]
            objects = list(set(objects))
            self.obj_clf = WordClassifier(use_phonemes=True)
            self.obj_clf.fit(objects)

            locations = [l["location"] for l in self.resource.locations]
            locations += [l["room"] for l in self.resource.locations]
            locations = list(set(locations))
            self.loc_clf = WordClassifier(use_phonemes=True)
            self.loc_clf.fit(locations)

    def align_object(self, cmd, valid, cmds, state, threshold=0.5):
        if "object" not in cmd.arguments:
            return cmd
        obj = cmd.arguments["object"]
        loc = cmd.arguments["location"]
        blacklist = ["it"]
        db = UnknownObjectDatabase()
        if self.align and obj and obj not in blacklist and db.get_properties(obj, loc) is None:
            try:
                inferred, confidence = self.obj_clf.infer(obj, with_proba=True)
                if confidence >= 1.0:
                    pass
                elif 1.0 > confidence > threshold:
                    rospy.loginfo("Aligned object: %s -> %s (confidence: %.2f)" % (obj, inferred, confidence))
                    cmd.add_argument("object", inferred)
                else:
                    rospy.logwarn("Unknown object?: %s (similarity: %.2f)" % (obj, confidence))
                    raise UnknownSymbolError("I don't know what is %s" % obj,
                                             cmd, valid, cmds, state)
            except UnknownSymbolError as e:
                raise e
            except Exception as e:
                raise UnknownSymbolError("Failed to align object %s: %s" % (obj, str(e)),
                                         cmd, valid, cmds, state)

        return cmd

    def align_location(self, cmd, valid, cmds, state, threshold=0.5):
        if "location" not in cmd.arguments:
            return cmd
        loc = cmd.arguments["location"]
        if self.align and loc and loc != self.initial_state["location"]:
            try:
                inferred, confidence = self.loc_clf.infer(loc, with_proba=True)
                if confidence >= 1.0:
                    pass
                elif confidence > threshold:
                    rospy.loginfo("Aligned location: %s -> %s (confidence: %.2f)" % (loc, inferred, confidence))
                    cmd.add_argument("location", inferred)
                else:
                    rospy.logwarn("Unknown location?: %s (confidence: %.2f)" % (loc, confidence))
                    raise UnknownSymbolError("I don't know where is %s" % loc,
                                             cmd, valid, cmds, state)
            except UnknownSymbolError as e:
                raise e
            except Exception as e:
                raise UnknownSymbolError("Failed to align location %s: %s" % (loc, str(e)),
                                         cmd, valid, cmds, state)

        return cmd

    def interpolate(self, cmds, ac_loader, max_depth=20):
        assert isinstance(ac_loader, ActionLoader)

        new_cmds = []
        for i, cmd in enumerate(cmds):
            if i == 0:
                new_cmds.append(cmd)
            elif cmd.name == "pick" and new_cmds[-1].name == "deliver":
                deliver = new_cmds.pop()
                if "object" in deliver.arguments:
                    cmd.add_argument("object", deliver.arguments["object"])
                    new_cmds.append(cmd)
                    new_cmds.append(deliver)
            else:
                new_cmds.append(cmd)
        cmds = new_cmds

        cmds = self.backtrack(cmds,
                              state=copy.deepcopy(self.initial_state),
                              valid=[],
                              depth=0,
                              max_depth=max_depth,
                              ac_loader=ac_loader)
        return cmds


    def backtrack(self, cmds, state, valid, depth, max_depth, ac_loader):
        if depth >= max_depth:
            raise OverflowError("search depth exceeds maximum: %d >= %d" % (depth, max_depth))
        if not cmds:
            return valid
        cmd = cmds[0]

        # align
        cmd = self.align_object(cmd, valid, cmds, state)
        cmd = self.align_location(cmd, valid, cmds, state)

        rospy.loginfo(">> %d: [%s] %s" % (depth, cmd.type, str(cmd)))
        rospy.loginfo("       %s" % state)

        # align 'it'
        if "object" in cmd.arguments:
            obj = cmd.arguments["object"]
            if obj.strip().lower() == 'it':
                if state["on_hand"]:
                    cmd.add_argument("object", state["on_hand"])
                elif state["near_object"]:
                    cmd.add_argument("object", state["near_object"])

        if cmd.type == 'go':
            if not cmd.arguments["location"]:
                raise CommandInterpolateError("I could not understand where to go",
                                              cmd, valid, cmds, state)
            state.update({'location': cmd.arguments["location"]})
        elif cmd.type == 'speak':
            if not cmd.arguments["content"]:
                raise CommandInterpolateError("I could not understand what to talk",
                                              cmd, valid, cmds, state)
            if not cmd.arguments["person"]:
                if state["near_person"]:
                    cmd.add_argument("person", state["near_person"])
                elif state["location"] == "operatorfront":
                    cmd.add_argument("person", "operator")
                else:
                    raise CommandInterpolateError("I don't know a person to talk with",
                                                  cmd, valid, cmds, state)
            if cmd.arguments["location"]:
                if state["location"] != cmd.arguments["location"]:
                    gocmd = ac_loader.gen_action("go")
                    gocmd.add_argument("location", cmd.arguments["location"])
                    return self.backtrack([gocmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
            else:
                cmd.add_argument("location", state["location"])
        elif cmd.type == 'find_object':
            if not cmd.arguments["object"] and not cmd.arguments["question"]:
                raise CommandInterpolateError("I could not know an object to find",
                                              cmd, valid, cmds, state)
            if cmd.arguments["location"]:
                if state["location"] != cmd.arguments["location"]:
                    gocmd = ac_loader.gen_action("go")
                    gocmd.add_argument("location", cmd.arguments["location"])
                    return self.backtrack([gocmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
            else:
                # check default location. if exists, go to default location
                # otherwise find at current position.
                obj_locs = {e["name"]: e["defaultLocation"] for e in self.resource.objects}
                if cmd.arguments["object"] in obj_locs.keys():
                    loc = obj_locs[cmd.arguments["object"]]
                    if state["location"] != loc:
                        gocmd = ac_loader.gen_action("go")
                        gocmd.add_argument("location", obj_locs[cmd.arguments["object"]])
                        return self.backtrack([gocmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
                    else:
                        cmd.add_argument("location", state["location"])
                else:
                    cmd.add_argument("location", state["location"])
            state.update({"near_object": cmd.arguments["object"]})
        elif cmd.type == 'find_person':
            if not cmd.arguments["person"] and not cmd.arguments["question"]:
                raise CommandInterpolateError("I could not know a person to find or order",
                                              cmd, valid, cmds, state)
            if cmd.arguments["location"]:
                if state["location"] != cmd.arguments["location"]:
                    gocmd = ac_loader.gen_action("go")
                    gocmd.add_argument("location", cmd.arguments["location"])
                    return self.backtrack([gocmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
            else:
                cmd.add_argument("location", state["location"])
            state.update({"near_person": cmd.arguments["person"] or "someone"})
        elif cmd.type == 'pick':
            # assert the object to pick
            if cmd.arguments["object"] == state["near_object"]:
                if not cmd.arguments["location"]:
                    cmd.add_argument("location", state["location"])
            elif not cmd.arguments["object"]:
                if state["near_object"]:
                    cmd.add_argument("object", state["near_object"])
                else:
                    raise CommandInterpolateError("I could not know an object to pick",
                                                  cmd, valid, cmds, state)

            # consider about location
            if cmd.arguments["location"]:
                if state["location"] != cmd.arguments["location"]:
                    gocmd = ac_loader.gen_action("go")
                    gocmd.add_argument("location", cmd.arguments["location"])
                    return self.backtrack([gocmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
            else:
                # check default location. if exists, go to default location
                # otherwise find at current position.
                obj_locs = {e["name"]: e["defaultLocation"] for e in self.resource.objects}

                if cmd.arguments["object"] in obj_locs:
                    loc = obj_locs[cmd.arguments["object"]]
                    if state["location"] != loc:
                        gocmd = ac_loader.gen_action("go")
                        gocmd.add_argument("location", obj_locs[cmd.arguments["object"]])
                        return self.backtrack([gocmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
                    else:
                        cmd.add_argument("location", state["location"])
                else:
                    # no default location found
                    raise CommandInterpolateError(
                        "I could not know where %s is located." % cmd.arguments["object"],
                        cmd, valid, cmds, state)

            state.update({"on_hand": cmd.arguments["object"],
                          "near_object": None})
        elif cmd.type == 'place':
            if cmd.arguments["location"]:
                if state["location"] != cmd.arguments["location"]:
                    gocmd = ac_loader.gen_action("go")
                    gocmd.add_argument("location", cmd.arguments["location"])
                    return self.backtrack([gocmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
            else:
                cmd.add_argument("location", state["location"])
            if not cmd.arguments["object"] or cmd.arguments["object"] == "it":
                if state["on_hand"]:
                    cmd.add_argument("object", state["on_hand"])
                else:
                    raise CommandInterpolateError("I could not know an object to place",
                                                  cmd, valid, cmds, state)
            state.update({"on_hand": None,
                          "near_object": cmd.arguments["object"]})
        elif cmd.type == 'deliver':
            # consider about object first
            if not state["on_hand"]:
                if not cmd.arguments["object"]:
                    raise CommandInterpolateError("I could not know an object to deliver",
                                                  cmd, valid, cmds, state)
                pickcmd = ac_loader.gen_action("pick")
                pickcmd.add_argument("object", cmd.arguments["object"])
                return self.backtrack([pickcmd] + cmds, state, valid, depth+1, max_depth, ac_loader)

            # consider about person to deliver?
            if not cmd.arguments["person"]:
                if state["near_person"]:
                    cmd.add_argument("person", state["near_person"])
                else:
                    raise CommandInterpolateError("I could not know a person to deliver",
                                                  cmd, valid, cmds, state)
            elif (cmd.arguments["person"].lower() == "me" and
                  state["location"] != self.initial_state["location"]):
                gocmd = ac_loader.gen_action("go")
                gocmd.add_argument("location", "operatorfront")
                return self.backtrack([gocmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
            # if not state["on_hand"]:
            #     if not cmd.arguments["object"]:
            #         raise CommandInterpolateError("I could not know an object to deliver",
            #                                       cmd, valid, cmds, state)
            #     pickcmd = ac_loader.gen_action("pick")
            #     pickcmd.add_argument("object", cmd.arguments["object"])
            #     return self.backtrack([pickcmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
            if cmd.arguments["location"]:
                if state["location"] != cmd.arguments["location"]:
                    # what does location means? the location of the person or the object?
                    gocmd = ac_loader.gen_action("go")
                    gocmd.add_argument("location", cmd.arguments["location"])
                    return self.backtrack([gocmd] + cmds, state, valid, depth+1, max_depth, ac_loader)
            else:
                cmd.add_argument("location", state["location"])
            state.update({"on_hand": None,
                          "near_object": cmd.arguments["object"]})
        elif cmd.type == 'follow':
            if not cmd.arguments["person"]:
                if state["near_person"]:
                    cmd.add_argument("person", state["near_person"])

        rospy.loginfo("<< %d: [%s] %s" % (depth, cmd.type, str(cmd)))
        rospy.loginfo("       %s" % state)

        return self.backtrack(cmds[1:], state, valid + [cmd], depth+1, max_depth, ac_loader)


if __name__ == '__main__':
    rospy.init_node("command_interpolator")
    from jsk_2017_home_butler.command_parser import CommandParser

    ac_loader = ActionLoader()

    rospy.set_param("~app_id", '48a5ef97-c9c5-4986-85c7-f6f5ef6f4bbc')
    rospy.set_param("~app_key", 'bfececf25171466a97a61d600245f4ef')

    parser = CommandParser()
    interp = CommandInterpolator()

    def interpolate(query):
        print "============================================"
        print "query:", query
        print
        print "recognizing & matching ambiguous words:"
        actions = parser.parse_command(query, ac_loader)

        for i, ac in enumerate(actions):
            print "#%s" % str(i+1), ac

        print
        print "interpolating and backtracking:"
        actions = interp.interpolate(actions, ac_loader)
        for i, ac in enumerate(actions):
            print "#%s" % str(i+1), ac

    # test
    queries = [
        "Navigate to the bathroom, find someone, and say your team's name",
        "Get the tea from the cupboard to the center table",
        "Answer a question to Hanna at the cabinet",
        "Navigate to the office, locate Alex, and tell something about yourself",
        "Deliver the manju to Samantha at the bed",
        "take the T to the center table",
        "Pick up the coke from the cupboard and deliver it to me",
    ]
    queries = [
#        "Could you please enter to the bedroom, look for Sophia, follow him, and tell a joke.",
#        "Could you please enter to the bedroom, look for hogehoge, follow him, and tell a joke."
#        "Could you bring the mochi to me from the kitchen"
        # "give me the mochi"
        "give me the candy",
    ]
    for query in queries:
        try:
            interpolate(query)
        except Exception as e:
            print "Error:", str(e)
            print "Command:", repr(e)
            import traceback
            traceback.print_exc()
