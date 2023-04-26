#include <fstream>
#include <iostream>
// #include <boost/functional/hash.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <list>
#include <queue>
#include <regex>
#include <set>
#include <stack>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

#define IN(Y, X) ((X).find(Y) != (X).end())
#define NOTIN(Y, X) ((X).find(Y) == (X).end())

#define TIME                                                                   \
  std::chrono::duration_cast<std::chrono::milliseconds>(                       \
      std::chrono::system_clock::now().time_since_epoch())                     \
      .count()

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

typedef unordered_set<GroundedCondition> state_t;
typedef pair<pair<pair<state_t, GroundedAction>, state_t>, pair<int, int>>
    node_t;

bool print_status = true;

class GroundedAction {
private:
  string name;
  list<string> arg_values;

public:
  GroundedAction(string name, list<string> arg_values) {
    this->name = name;
    for (string ar : arg_values) {
      this->arg_values.push_back(ar);
    }
  }
  GroundedAction(const GroundedAction &gc) {
    this->name = gc.name;
    for (string ar : gc.arg_values) {
      this->arg_values.push_back(ar);
    }
  }
  GroundedAction() {}

  string get_name() const { return this->name; }

  list<string> get_arg_values() const { return this->arg_values; }

  bool operator==(const GroundedAction &rhs) const {
    if (this->name != rhs.name ||
        this->arg_values.size() != rhs.arg_values.size())
      return false;

    auto lhs_it = this->arg_values.begin();
    auto rhs_it = rhs.arg_values.begin();

    while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end()) {
      if (*lhs_it != *rhs_it)
        return false;
      ++lhs_it;
      ++rhs_it;
    }
    return true;
  }

  friend ostream &operator<<(ostream &os, const GroundedAction &gac) {
    os << gac.toString() << " ";
    return os;
  }

  string toString() const {
    string temp = "";
    temp += this->name;
    temp += "(";
    for (string l : this->arg_values) {
      temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
  }
};

class GroundedCondition {
private:
  string predicate;
  list<string> arg_values;
  bool truth = true;

public:
  GroundedCondition(string predicate, list<string> arg_values,
                    bool truth = true) {
    this->predicate = predicate;
    this->truth = truth; // fixed
    for (string l : arg_values) {
      this->arg_values.push_back(l);
    }
  }

  GroundedCondition(const GroundedCondition &gc) {
    this->predicate = gc.predicate;
    this->truth = gc.truth; // fixed
    for (string l : gc.arg_values) {
      this->arg_values.push_back(l);
    }
  }

  string get_predicate() const { return this->predicate; }
  list<string> get_arg_values() const { return this->arg_values; }

  bool get_truth() const { return this->truth; }

  friend ostream &operator<<(ostream &os, const GroundedCondition &pred) {
    os << pred.toString() << " ";
    return os;
  }

  bool operator==(const GroundedCondition &rhs) const {
    if (this->predicate != rhs.predicate ||
        this->arg_values.size() != rhs.arg_values.size())
      return false;

    auto lhs_it = this->arg_values.begin();
    auto rhs_it = rhs.arg_values.begin();

    while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end()) {
      if (*lhs_it != *rhs_it)
        return false;
      ++lhs_it;
      ++rhs_it;
    }

    if (this->truth != rhs.get_truth()) // fixed
      return false;

    return true;
  }

  string toString() const {
    string temp = this->truth ? "" : "!";
    temp += this->predicate;
    temp += "(";
    for (string l : this->arg_values) {
      temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
  }

  /********/
  inline GroundedCondition negate() const {
    return GroundedCondition(this->predicate, this->arg_values, !this->truth);
  }

  inline bool is_negate(GroundedCondition condition) const {
    list<string> other_args = condition.get_arg_values();
    if (condition.predicate != this->predicate) {
      return false;
    }
    if (this->truth == condition.get_truth()) {
      return false;
    }
    auto other_itr = other_args.begin();
    auto this_itr = this->arg_values.begin();
    while (other_itr != other_args.end()) {
      if (*other_itr != *this_itr) {
        return false;
      }
      other_itr++;
      this_itr++;
    }
    return true;
  }
};

struct GroundedConditionComparator {
  bool operator()(const GroundedCondition &lhs,
                  const GroundedCondition &rhs) const {
    return lhs == rhs;
  }
};

struct GroundedConditionHasher {
  size_t operator()(const GroundedCondition &gcond) const {
    return hash<string>{}(gcond.toString());
  }
};

namespace std {
template <> struct hash<GroundedCondition> {
  size_t operator()(const GroundedCondition &gcond) const {
    return hash<string>{}(gcond.toString());
  }
};
} // namespace std

class Condition {
private:
  string predicate;
  list<string> args;
  bool truth;

public:
  Condition(string pred, list<string> args, bool truth) {
    this->predicate = pred;
    this->truth = truth;
    for (string ar : args) {
      this->args.push_back(ar);
    }
  }

  string get_predicate() const { return this->predicate; }

  list<string> get_args() const { return this->args; }

  bool get_truth() const { return this->truth; }

  friend ostream &operator<<(ostream &os, const Condition &cond) {
    os << cond.toString() << " ";
    return os;
  }

  bool operator==(const Condition &rhs) const // fixed
  {

    if (this->predicate != rhs.predicate ||
        this->args.size() != rhs.args.size())
      return false;

    auto lhs_it = this->args.begin();
    auto rhs_it = rhs.args.begin();

    while (lhs_it != this->args.end() && rhs_it != rhs.args.end()) {
      if (*lhs_it != *rhs_it)
        return false;
      ++lhs_it;
      ++rhs_it;
    }

    if (this->truth != rhs.get_truth())
      return false;

    return true;
  }

  string toString() const {
    string temp = "";
    if (!this->truth)
      temp += "!";
    temp += this->predicate;
    temp += "(";
    for (string l : this->args) {
      temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
  }

  /********/
  inline GroundedCondition
  ground(const unordered_map<string, string> &bindings) const {
    list<string> arg_list;
    for (string s : this->args) {
      arg_list.push_back(bindings.at(s));
    }
    return GroundedCondition(this->predicate, arg_list, this->truth);
  }

  inline vector<unordered_map<string, string>>
  match(const unordered_map<string, string> &bindings, const state_t &state,
        const unordered_set<string> &symbols) const {

    vector<string> args;
    for (string s : this->args) {
      if (NOTIN(s, bindings)) {
        args.push_back(s);
      }
    }
    int num_args = args.size();
    vector<unordered_map<string, string>> possible;
    unordered_set<string> used;
    for (auto item : bindings) {
      if (item.first != item.second) {
        used.insert(item.second);
      }
    }

    stack<pair<unordered_map<string, string>, int>> partial_bindings;
    partial_bindings.push(make_pair(bindings, 0));
    while (partial_bindings.size()) {
      pair<unordered_map<string, string>, int> item = partial_bindings.top();
      partial_bindings.pop();
      unordered_map<string, string> partial = item.first;
      int index = item.second;

      if (index == num_args) {
        // got complete binding
        GroundedCondition condition = this->ground(partial);
        if (IN(condition, state)) {
          possible.push_back(partial);
        }
      } else {
        int new_index = index + 1;
        for (string sym : symbols) {
          if (NOTIN(sym, used)) {
            unordered_map<string, string> new_bindings = partial;
            new_bindings.insert({args[index], sym});
            partial_bindings.push(make_pair(new_bindings, new_index));
          }
        }
      }
    }
    return possible;
  }
};

struct ConditionComparator {
  bool operator()(const Condition &lhs, const Condition &rhs) const {
    return lhs == rhs;
  }
};

struct ConditionHasher {
  size_t operator()(const Condition &cond) const {
    return hash<string>{}(cond.toString());
  }
};

namespace std {
template <> struct hash<Condition> {
  size_t operator()(const Condition &cond) const {
    return hash<string>{}(cond.toString());
  }
};
} // namespace std

class Action {
private:
  string name;
  list<string> args;
  unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
  unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
  Action(
      string name, list<string> args,
      unordered_set<Condition, ConditionHasher, ConditionComparator>
          &preconditions,
      unordered_set<Condition, ConditionHasher, ConditionComparator> &effects) {
    this->name = name;
    for (string l : args) {
      this->args.push_back(l);
    }
    for (Condition pc : preconditions) {
      this->preconditions.insert(pc);
    }
    for (Condition pc : effects) {
      this->effects.insert(pc);
    }
  }
  string get_name() const { return this->name; }
  list<string> get_args() const { return this->args; }
  unordered_set<Condition, ConditionHasher, ConditionComparator>
  get_preconditions() const {
    return this->preconditions;
  }
  unordered_set<Condition, ConditionHasher, ConditionComparator>
  get_effects() const {
    return this->effects;
  }

  bool operator==(const Action &rhs) const {
    if (this->get_name() != rhs.get_name() ||
        this->get_args().size() != rhs.get_args().size())
      return false;

    return true;
  }

  friend ostream &operator<<(ostream &os, const Action &ac) {
    os << ac.toString() << endl;
    os << "Precondition: ";
    for (Condition precond : ac.get_preconditions())
      os << precond;
    os << endl;
    os << "Effect: ";
    for (Condition effect : ac.get_effects())
      os << effect;
    os << endl;
    return os;
  }

  string toString() const {
    string temp = "";
    temp += this->get_name();
    temp += "(";
    for (string l : this->get_args()) {
      temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
  }

  /*********/
  inline GroundedAction ground(const unordered_map<string, string> &binding,
                               state_t &effects) {
    list<string> arg_values;
    for (string s : this->args) {
      arg_values.push_back(binding.at(s));
    }
    effects.clear();
    for (Condition condition : this->effects) {
      effects.insert(condition.ground(binding));
    }
    return GroundedAction(this->name, arg_values);
  }
};

struct ActionComparator {
  bool operator()(const Action &lhs, const Action &rhs) const {
    return lhs == rhs;
  }
};

struct ActionHasher {
  size_t operator()(const Action &ac) const {
    return hash<string>{}(ac.get_name());
  }
};

namespace std {
template <> struct hash<Action> {
  size_t operator()(const Action &ac) const {
    return hash<string>{}(ac.get_name());
  }
};
} // namespace std

class Env {
private:
  unordered_set<GroundedCondition, GroundedConditionHasher,
                GroundedConditionComparator>
      initial_conditions;
  unordered_set<GroundedCondition, GroundedConditionHasher,
                GroundedConditionComparator>
      goal_conditions;
  unordered_set<Action, ActionHasher, ActionComparator> actions;
  unordered_set<string> symbols;

public:
  void remove_initial_condition(GroundedCondition gc) {
    this->initial_conditions.erase(gc);
  }
  void add_initial_condition(GroundedCondition gc) {
    this->initial_conditions.insert(gc);
  }
  void add_goal_condition(GroundedCondition gc) {
    this->goal_conditions.insert(gc);
  }
  void remove_goal_condition(GroundedCondition gc) {
    this->goal_conditions.erase(gc);
  }
  void add_symbol(string symbol) { symbols.insert(symbol); }
  void add_symbols(list<string> symbols) {
    for (string l : symbols)
      this->symbols.insert(l);
  }
  void add_action(Action action) { this->actions.insert(action); }

  Action get_action(string name) {
    for (Action a : this->actions) {
      if (a.get_name() == name)
        return a;
    }
    throw runtime_error("Action " + name + " not found!");
  }
  unordered_set<string> get_symbols() const { return this->symbols; }
  inline const unordered_set<Action, ActionHasher, ActionComparator> &
  get_actions() const {
    return this->actions;
  }
  inline const unordered_set<GroundedCondition, GroundedConditionHasher,
                             GroundedConditionComparator>
  get_initial_conditions() const {
    return this->initial_conditions;
  }
  inline const unordered_set<GroundedCondition, GroundedConditionHasher,
                             GroundedConditionComparator>
  get_goal_conditions() const {
    return this->goal_conditions;
  }

  friend ostream &operator<<(ostream &os, const Env &w) {
    os << "***** Environment *****" << endl << endl;
    os << "Symbols: ";
    for (string s : w.get_symbols())
      os << s + ",";
    os << endl;
    os << "Initial conditions: ";
    for (GroundedCondition s : w.initial_conditions)
      os << s;
    os << endl;
    os << "Goal conditions: ";
    for (GroundedCondition g : w.goal_conditions)
      os << g;
    os << endl;
    os << "Actions:" << endl;
    for (Action g : w.actions)
      os << g << endl;
    cout << "***** Environment Created! *****" << endl;
    return os;
  }
};

namespace std {
ostream &operator<<(std::ostream &os, const state_t &state) {
  os << "state: ";
  for (GroundedCondition condition : state) {
    os << condition << " ";
  }
  return os;
}
} // namespace std

vector<pair<GroundedAction, state_t>> get_actions(const state_t &state,
                                                  const Env *env) {

  vector<pair<GroundedAction, state_t>> all_actions;
  unordered_map<string, vector<GroundedCondition>> conditions_by_name;
  for (GroundedCondition condition : state) {
    string predicate = condition.get_predicate();
    if (NOTIN(predicate, conditions_by_name)) {
      conditions_by_name.insert({predicate, vector<GroundedCondition>()});
    }
    conditions_by_name.at(predicate).push_back(condition);
  }
  unordered_map<string, string> bindings_base;
  for (string sym : env->get_symbols()) {
    bindings_base.insert({sym, sym});
  }

  for (Action action : env->get_actions()) {
    vector<Condition> conditions;
    for (Condition condition : action.get_preconditions()) {
      conditions.push_back(condition);
    }
    int num_conditions = conditions.size();

    stack<pair<unordered_map<string, string>, int>> partial_bindings;
    partial_bindings.push(make_pair(bindings_base, 0));
    while (partial_bindings.size()) {
      pair<unordered_map<string, string>, int> item = partial_bindings.top();
      partial_bindings.pop();
      unordered_map<string, string> partial = item.first;
      int index = item.second;

      if (index == num_conditions) {
        // got complete binding
        state_t new_state;
        GroundedAction grounded = action.ground(partial, new_state);
        all_actions.push_back(make_pair(grounded, new_state));
      } else {
        vector<unordered_map<string, string>> new_bindings =
            conditions[index].match(partial, state, env->get_symbols());
        int new_index = index + 1;
        for (unordered_map<string, string> new_binding : new_bindings) {
          partial_bindings.push(make_pair(new_binding, new_index));
        }
      }
    }
  }

  return all_actions;
}

class GraphPlanLayer {
private:
  const state_t *prev_conditions;
  state_t conditions;

public:
  GraphPlanLayer() {}
  GraphPlanLayer(const state_t *prev_conditions) {
    this->prev_conditions = prev_conditions;
    this->conditions = state_t();

    // add no-ops
    // cout << "adding no-ops" << endl;
    for (GroundedCondition condition : *prev_conditions) {
      this->conditions.insert(condition);
    }
  }
  inline const state_t &get_conditions() const { return this->conditions; }
  inline bool contains(const state_t &conditions) const {
    for (GroundedCondition condition : conditions) {
      if (NOTIN(condition, this->conditions)) {
        return false;
      }
    }
    return true;
  }

  inline void add_actions(const Env *env) {
    vector<pair<GroundedAction, state_t>> grounded_actions =
        get_actions(*this->prev_conditions, env);

    for (pair<GroundedAction, state_t> grounded : grounded_actions) {
      for (GroundedCondition condition : grounded.second) {
        this->conditions.insert(condition);
      }
    }
  }

  friend ostream &operator<<(ostream &os, const GraphPlanLayer &GP) {
    os << "GraphPlanLayer @ " << &GP << endl;
    os << "total conditions: " << GP.conditions.size() << endl;
    int index = 0;
    for (GroundedCondition condition : GP.conditions) {
      os << "  " << condition << endl;
      os << "--------------------" << endl;
    }
    return os;
  }
};

list<string> parse_symbols(string symbols_str) {
  list<string> symbols;
  size_t pos = 0;
  string delimiter = ",";
  while ((pos = symbols_str.find(delimiter)) != string::npos) {
    string symbol = symbols_str.substr(0, pos);
    symbols_str.erase(0, pos + delimiter.length());
    symbols.push_back(symbol);
  }
  symbols.push_back(symbols_str);
  return symbols;
}

Env *create_env(char *filename) {
  ifstream input_file(filename);
  Env *env = new Env();
  regex symbolStateRegex("symbols:", regex::icase);
  regex symbolRegex("([a-zA-Z0-9_, ]+) *");
  regex initialConditionRegex("initialconditions:(.*)", regex::icase);
  regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
  regex goalConditionRegex("goalconditions:(.*)", regex::icase);
  regex actionRegex("actions:", regex::icase);
  regex precondRegex("preconditions:(.*)", regex::icase);
  regex effectRegex("effects:(.*)", regex::icase);
  int parser = SYMBOLS;

  unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
  unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
  string action_name;
  string action_args;

  string line;
  if (input_file.is_open()) {
    while (getline(input_file, line)) {
      string::iterator end_pos = remove(line.begin(), line.end(), ' ');
      line.erase(end_pos, line.end());

      if (line == "")
        continue;

      if (parser == SYMBOLS) {
        smatch results;
        if (regex_search(line, results, symbolStateRegex)) {
          line = line.substr(8);
          sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
          sregex_token_iterator end;

          env->add_symbols(parse_symbols(iter->str())); // fixed

          parser = INITIAL;
        } else {
          cout << "Symbols are not specified correctly." << endl;
          throw;
        }
      } else if (parser == INITIAL) {
        const char *line_c = line.c_str();
        if (regex_match(line_c, initialConditionRegex)) {
          const std::vector<int> submatches = {1, 2};
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex,
                                     submatches);
          sregex_token_iterator end;

          while (iter != end) {
            // name
            string predicate = iter->str();
            iter++;
            // args
            string args = iter->str();
            iter++;

            if (predicate[0] == '!') {
              env->remove_initial_condition(
                  GroundedCondition(predicate.substr(1), parse_symbols(args)));
            } else {
              env->add_initial_condition(
                  GroundedCondition(predicate, parse_symbols(args)));
            }
          }

          parser = GOAL;
        } else {
          cout << "Initial conditions not specified correctly." << endl;
          throw;
        }
      } else if (parser == GOAL) {
        const char *line_c = line.c_str();
        if (regex_match(line_c, goalConditionRegex)) {
          const std::vector<int> submatches = {1, 2};
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex,
                                     submatches);
          sregex_token_iterator end;

          while (iter != end) {
            // name
            string predicate = iter->str();
            iter++;
            // args
            string args = iter->str();
            iter++;

            if (predicate[0] == '!') {
              env->remove_goal_condition(
                  GroundedCondition(predicate.substr(1), parse_symbols(args)));
            } else {
              env->add_goal_condition(
                  GroundedCondition(predicate, parse_symbols(args)));
            }
          }

          parser = ACTIONS;
        } else {
          cout << "Goal conditions not specified correctly." << endl;
          throw;
        }
      } else if (parser == ACTIONS) {
        const char *line_c = line.c_str();
        if (regex_match(line_c, actionRegex)) {
          parser = ACTION_DEFINITION;
        } else {
          cout << "Actions not specified correctly." << endl;
          throw;
        }
      } else if (parser == ACTION_DEFINITION) {
        const char *line_c = line.c_str();
        if (regex_match(line_c, conditionRegex)) {
          const std::vector<int> submatches = {1, 2};
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex,
                                     submatches);
          sregex_token_iterator end;
          // name
          action_name = iter->str();
          iter++;
          // args
          action_args = iter->str();
          iter++;

          parser = ACTION_PRECONDITION;
        } else {
          cout << "Action not specified correctly." << endl;
          cout << line_c << endl;
          throw;
        }
      } else if (parser == ACTION_PRECONDITION) {
        const char *line_c = line.c_str();
        if (regex_match(line_c, precondRegex)) {
          const std::vector<int> submatches = {1, 2};
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex,
                                     submatches);
          sregex_token_iterator end;

          while (iter != end) {
            // name
            string predicate = iter->str();
            iter++;
            // args
            string args = iter->str();
            iter++;

            bool truth;

            if (predicate[0] == '!') {
              predicate = predicate.substr(1);
              truth = false;
            } else {
              truth = true;
            }

            Condition precond(predicate, parse_symbols(args), truth);
            preconditions.insert(precond);
          }

          parser = ACTION_EFFECT;
        } else {
          cout << "Precondition not specified correctly." << endl;
          throw;
        }
      } else if (parser == ACTION_EFFECT) {
        const char *line_c = line.c_str();
        if (regex_match(line_c, effectRegex)) {
          const std::vector<int> submatches = {1, 2};
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex,
                                     submatches);
          sregex_token_iterator end;

          while (iter != end) {
            // name
            string predicate = iter->str();
            iter++;
            // args
            string args = iter->str();
            iter++;

            bool truth;

            if (predicate[0] == '!') {
              predicate = predicate.substr(1);
              truth = false;
            } else {
              truth = true;
            }

            Condition effect(predicate, parse_symbols(args), truth);
            effects.insert(effect);
          }

          env->add_action(Action(action_name, parse_symbols(action_args),
                                 preconditions, effects));

          preconditions.clear();
          effects.clear();
          parser = ACTION_DEFINITION;
        } else {
          cout << "Effects not specified correctly." << endl;
          throw;
        }
      }
    }
    input_file.close();
  }

  else
    cout << "Unable to open file";

  return env;
}

namespace std {
template <> struct hash<state_t> {
  size_t operator()(const state_t &state) const {
    size_t temp = state.size();
    for (const GroundedCondition condition : state) {
      temp ^= std::hash<GroundedCondition>{}(condition);
    }
    return temp;
  }
};
template <> struct hash<pair<state_t, size_t>> {
  size_t operator()(const pair<state_t, size_t> &cached) const {
    return cached.second;
  }
};
bool operator==(const state_t s1, const state_t s2) {
  if (s1.size() != s2.size()) {
    return false;
  }
  for (GroundedCondition condition : s1) {
    if (NOTIN(condition, s2)) {
      return false;
    }
  }
  return true;
}
bool operator<(const node_t &lhs, const node_t &rhs) {
  // default priority_queue is max heap
  return lhs.second.second > rhs.second.second ||
         (lhs.second.second == rhs.second.second &&
          lhs.second.first < rhs.second.first);
}
} // namespace std

static unordered_map<pair<state_t, size_t>, int> heuristics_multi;
static inline int get_heuristic_multi(const state_t &start, const state_t &goal,
                                      const Env *env) {
  pair<state_t, size_t> augmented = make_pair(start, hash<state_t>{}(start));
  if (NOTIN(augmented, heuristics_multi)) {
    bool isgoal = true;
    for (GroundedCondition condition : goal) {
      if (NOTIN(condition, start)) {
        isgoal = false;
        break;
      }
    }
    int res = 0;
    if (!isgoal) {
      GraphPlanLayer *layer = new GraphPlanLayer(&start);
      layer->add_actions(env);
      res = get_heuristic_multi(layer->get_conditions(), goal, env) + 1;
      delete layer;
    }
    heuristics_multi[augmented] = res;
    return res;
  } else {
    return heuristics_multi.at(augmented);
  }
}

static unordered_map<pair<state_t, size_t>, int> heuristics_naive;
static inline int get_heuristic_naive(const state_t &start, const state_t &goal,
                                      const Env *env) {
  pair<state_t, size_t> augmented = make_pair(start, hash<state_t>{}(start));
  if (NOTIN(augmented, heuristics_multi)) {
    int res = 0;
    for (GroundedCondition condition : goal) {
      if (NOTIN(condition, start)) {
        res++;
      }
    }
    heuristics_naive[augmented] = res;
    return res;
  } else {
    return heuristics_naive.at(augmented);
  }
}

list<GroundedAction> planner(Env *env) {
  // this is where you insert your planner

  list<GroundedAction> plan;

  unordered_map<state_t, pair<state_t, GroundedAction>> parents;
  unordered_map<state_t, int> cost;
  priority_queue<node_t> queue;

  state_t start, goal;
  for (GroundedCondition condition : env->get_initial_conditions()) {
    start.insert(condition);
  }
  for (GroundedCondition condition : env->get_goal_conditions()) {
    goal.insert(condition);
  }

  node_t start_node = make_pair(
      make_pair(make_pair(start, GroundedAction()), start), make_pair(0, 0));
  cost[start] = -1;
  queue.push(start_node);
  int count = 0;
  while (queue.size()) {
    node_t node = queue.top();
    queue.pop();
    state_t parent = node.first.first.first;
    GroundedAction last_action = node.first.first.second;
    state_t state = node.first.second;
    count++;

    if (IN(state, parents)) {
      continue;
    }

    parents[state] = make_pair(parent, last_action);
    cost[state] = cost[parent] + 1;

    // cout << "got state: " << state << endl;
    // cout << " with heuristic value " << get_heuristic_naive(state, goal, env)
    //      << endl;
    // cout << " and cost value " << cost.at(state) << endl;

    // goal test
    if (get_heuristic_naive(state, goal, env) == 0) {
      cout << "Found goal after " << count << " nodes" << endl;
      cout << heuristics_naive.size() << " heuristics computed" << endl;
      while (!(state == start)) {
        // cout << "Back tracking from state " << state << endl;
        // cout << " with heuristic value "
        //      << get_heuristic_multi(state, goal, env) << endl;
        // cout << " and cost value " << cost.at(state) << endl;
        pair<state_t, GroundedAction> item = parents.at(state);
        plan.push_front(item.second);
        state = item.first;
        // cout << "Got action: " << item.second << endl;
      }
      break;
    }

    vector<pair<GroundedAction, state_t>> grounded_actions =
        get_actions(state, env);

    for (pair<GroundedAction, state_t> grounded : grounded_actions) {
      state_t new_conditions = grounded.second;

      state_t new_state = state;
      for (GroundedCondition condition : new_conditions) {
        if (condition.get_truth()) {
          new_state.insert(condition);
        } else {
          GroundedCondition negate = condition.negate();
          if (IN(negate, new_state)) {
            new_state.erase(negate);
          }
        }
      }

      // cout << "old state: " << state << endl;
      // cout << "action: " << grounded.first << endl;
      // cout << "new state: " << new_state << endl << endl;

      // int heuristic = 0;
      int heuristic = get_heuristic_naive(new_state, goal, env);
      node_t new_node =
          make_pair(make_pair(make_pair(state, grounded.first), new_state),
                    make_pair(cost.at(state), cost.at(state) + heuristic + 1));
      queue.push(new_node);
    }
  }

  return plan;
}

int main(int argc, char *argv[]) {
  // DO NOT CHANGE THIS FUNCTION
  char *filename = (char *)("example.txt");
  if (argc > 1)
    filename = argv[1];

  cout << "Environment: " << filename << endl << endl;
  Env *env = create_env(filename);
  if (print_status) {
    cout << *env;
  }

  list<GroundedAction> actions = planner(env);

  cout << "\nPlan: " << endl;
  for (GroundedAction gac : actions) {
    cout << gac << endl;
  }

  return 0;
}
