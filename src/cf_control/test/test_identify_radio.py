from cf_control.config import DRONES
from cf_control.identify_radio import _resolve_uri, blink_identity
import pytest


def test_resolve_uri_accepts_configured_name_and_explicit_uri():
    explicit_uri = 'radio://0/80/2M/E7E7E7E7AA'

    assert _resolve_uri('cf1') == DRONES['cf1']
    assert _resolve_uri(explicit_uri) == explicit_uri


def test_resolve_uri_rejects_unknown_name():
    with pytest.raises(ValueError, match='Unknown drone'):
        _resolve_uri('missing')


def test_blink_identity_rejects_unsafe_duration_before_radio_access():
    with pytest.raises(ValueError, match='at most 60'):
        blink_identity('radio://0/80/2M/E7E7E7E7AA', 61.0)
