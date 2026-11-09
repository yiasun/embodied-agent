"""Provider contracts. Model text is always untrusted until local validation."""
from dataclasses import dataclass, field, asdict
from typing import Any, Protocol


class PlanningError(ValueError):
    """Only fixed codes cross the boundary; never server messages or credentials."""
    def __init__(self, code, *, http_status=None):
        self.code = code
        self.http_status = http_status
        super().__init__(code)

    def as_dict(self):
        return {'error': self.code, 'http_status': self.http_status}


@dataclass(frozen=True)
class ProviderResult:
    content: str = field(repr=False)
    raw_response: Any = field(repr=False)
    provider: str
    model: str
    requested_model: str
    request_id: str | None
    input_tokens: int | None
    output_tokens: int | None
    total_tokens: int | None
    duration_seconds: float

    def as_dict(self):
        return asdict(self)


class Provider(Protocol):
    def generate_intent(self, instruction: str, scene: dict) -> ProviderResult: ...
