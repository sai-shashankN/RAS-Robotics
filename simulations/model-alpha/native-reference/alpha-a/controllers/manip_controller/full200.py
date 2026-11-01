"""Preserved live subscriber plus a distinct audit-only actual task endpoint."""
import json
from observed200 import ObservedAlpha200
from audit_reference import capture
class FullObservedAlpha200(ObservedAlpha200):
    def close(self,t):
        (self.out/'raw200-endpoint.json').write_text(json.dumps(capture(self.body,t),allow_nan=False),encoding='utf-8')
        super().close(t)
