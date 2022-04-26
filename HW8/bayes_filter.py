from typing import Dict, List, Tuple

class BayesFilter:
    def __init__(self, initial_belief: Dict) -> None:
        self.initial_belief = initial_belief
        self.states = ["open", "closed"]
        self.action = None
        self.measurement = None

    def set_action_measurement(self, action: str, measurement: str) -> None:
        self.action = action
        self.measurement = measurement

    # sensor model
    @staticmethod
    def get_sensor_prediction(sensed_state: str, actual_state: str) -> float:
        if sensed_state == "open" and actual_state == "open":
            return 0.6
        if sensed_state == "closed" and actual_state == "open":
            return 0.4
        if sensed_state == "open" and actual_state == "closed":
            return 0.2
        if sensed_state == "closed" and actual_state == "closed":
            return 0.8

    # dynamic model/system model. Order: x(t), u(t), x(t-1)
    @staticmethod
    def get_system_prediction(curr:str, act:str, prev:str) -> float:
        if curr == "open" and act == "push" and prev == "open":
            return 1
        if curr == "closed" and act == "push" and prev == "open":
            return 0
        if curr == "open" and act == "push" and prev == "closed":
            return 0.8
        if curr == "closed" and act == "push" and prev == "closed":
            return 0.2
        if curr == "open" and act == "do_nothing" and prev == "open":
            return 1
        if curr == "closed" and act == "do_nothing" and prev == "open":
            return 0
        if curr == "open" and act == "do_nothing" and prev == "closed":
            return 0
        if curr == "closed" and act == "do_nothing" and prev == "closed":
            return 1

    def get_bel_prediction(self, state: str) -> float:
        prediction = 0
        for prev_state in self.states:
            # sys_p(open)*init_bel(open) + sys_prob(closed)*init_bel(closed)
            sys_prediction = self.get_system_prediction(state, self.action, prev_state)
            initial_belief = self.initial_belief[prev_state]
            prediction += sys_prediction * initial_belief
        return prediction

    def filter(self) -> Tuple[float, float]:
        bel_bar_open = self.get_bel_prediction("open")
        bel_open = self.get_sensor_prediction(self.measurement, "open") * bel_bar_open

        bel_bar_closed = self.get_bel_prediction("closed")
        bel_closed = self.get_sensor_prediction(self.measurement, "closed") * bel_bar_closed

        eta = 1 / (bel_open + bel_closed)

        return eta * bel_open, eta * bel_closed

    def filter_consecutive(self, inputs: List[Tuple[str,str]]) -> Tuple[float, float]:
        bel_open, bel_closed = 0.0, 0.0
        for (action, measurement) in inputs:
            bayes.set_action_measurement(action, measurement)
            bel_open, bel_closed = bayes.filter()

            # Update initial belief based on observation
            self.initial_belief = {"open": bel_open, "closed": bel_closed}

        return bel_open, bel_closed

if __name__ == "__main__":
    iterations = [
        # ("do_nothing", "closed"),
        # ("do_nothing", "closed"),
        ("push", "closed"),
        # ("do_nothing", "closed"),
        # ("push", "open"),
        # ("do_nothing", "open"),
    ]

    # Test individual (Comment out when testing consecutive inputs)
    bayes = BayesFilter(initial_belief={"open": 0.5, "closed": 0.5})
    for (act, measure) in iterations:
        bayes.set_action_measurement(act, measure)
        p_open, p_closed = bayes.filter()
        print("Probability that the door is open:", p_open)
        print("Probability that the door is closed", p_closed)

    # Test consecutively (Comment out when testing individual inputs)
    # p_open, p_closed = bayes.filter_consecutive(iterations)
    # print("Probability that the door is open:", p_open)
    # print("Probability that the door is closed", p_closed)
