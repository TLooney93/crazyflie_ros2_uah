from cf_control.gesture_trial_logger import summarize_trials


def test_summarize_trials_counts_each_gesture():
    rows = [
        {'intended_gesture': 'takeoff', 'correct': True},
        {'intended_gesture': 'takeoff', 'correct': False},
        {'intended_gesture': 'land', 'correct': True},
    ]

    assert summarize_trials(rows) == {
        'takeoff': (1, 2),
        'land': (1, 1),
    }
