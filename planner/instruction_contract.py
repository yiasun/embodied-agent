"""A bounded navigation language, NOT general natural-language understanding.

Grammar v1: comma/ideographic-comma/semicolon separated navigation clauses;
optional ordering markers, one navigation verb, one exact literal location per
clause. Quoted labels are opaque data; unquoted labels exclude control words.
Nothing is extracted from an unsupported clause and no partial task is returned.
"""
import re
from dataclasses import dataclass

from .providers.base import PlanningError
from .validation import MIN_TASKS, MAX_TASKS

GRAMMAR_VERSION = 'explicit-navigation-v1'
SEPARATORS = frozenset('，,、;；')
QUOTES = {'"': '"', '“': '”', '「': '」'}
QUOTE_CHARS = frozenset(''.join(QUOTES) + ''.join(QUOTES.values()))
CLAUSE = re.compile(r'(?:(先|再|然后|接着|最后)\s*)?(?:前往|到达|去|到)\s*(.+)')
# These are syntax-reserved words, not an attempt to infer arbitrary semantics.
# A registry label containing them must be explicitly quoted as a literal.
BARE_RESERVED = ('不', '别', '勿', '去', '前往', '到达', '然后', '接着', '最后',
                 '再', '先', '并', '或', '但', '如果', '否则')


@dataclass(frozen=True)
class NavigationContract:
    locations: tuple[str, ...]
    grammar: str = GRAMMAR_VERSION

    def steps(self):
        return [{'action': 'NAVIGATE', 'location': location} for location in self.locations]


def _syntax_error():
    # Preserve the existing public parse-error code; callers should ask for a
    # complete, explicit instruction rather than silently invoking an LLM.
    raise PlanningError('RULE_PARSE_ERROR')


def _split_clauses(text):
    parts, current, closing = [], [], None
    for char in text:
        if closing is not None:
            if char == closing:
                closing = None
            elif char in QUOTE_CHARS:
                _syntax_error()
            current.append(char)
        elif char in QUOTES:
            closing = QUOTES[char]
            current.append(char)
        elif char in QUOTE_CHARS:
            _syntax_error()
        elif char in SEPARATORS:
            parts.append(''.join(current).strip())
            current = []
        else:
            current.append(char)
    if closing is not None:
        _syntax_error()
    parts.append(''.join(current).strip())
    return parts


def _literal(raw):
    if not raw:
        _syntax_error()
    if raw[0] in QUOTES:
        if len(raw) < 3 or raw[-1] != QUOTES[raw[0]]:
            _syntax_error()
        label = raw[1:-1]
        if not label or any(c in QUOTE_CHARS or ord(c) < 32 for c in label):
            _syntax_error()
        return label
    if (not all(c.isalnum() or c in '_-' for c in raw)
            or any(word in raw for word in BARE_RESERVED)):
        _syntax_error()
    return raw


def resolve_navigation_instruction(instruction, scene):
    from .intent_validator import validate_input
    validate_input(instruction, scene)
    text = instruction.strip()
    if '\n' in text or '\r' in text:
        _syntax_error()
    if text[-1] in '。.':
        text = text[:-1].rstrip()
    clauses = _split_clauses(text)
    if not MIN_TASKS <= len(clauses) <= MAX_TASKS:
        raise PlanningError('INVALID_TASK_COUNT')
    locations = []
    for index, clause in enumerate(clauses):
        match = CLAUSE.fullmatch(clause)
        if match is None:
            _syntax_error()
        marker, raw = match.groups()
        if ((marker == '先' and index != 0)
                or (marker in ('再', '然后', '接着') and index == 0)
                or (marker == '最后' and (index != len(clauses) - 1 or len(clauses) == 1))):
            _syntax_error()
        locations.append(_literal(raw.strip()))
    # Check every fully parsed literal before invoking any provider. Do not
    # filter unknown labels or deduplicate the requested sequence.
    if any(location not in scene['waypoints'] for location in locations):
        raise PlanningError('UNKNOWN_LOCATION')
    return NavigationContract(tuple(locations))
