"""Exact JSON lexical-token comparison, ignoring only external ASCII whitespace.

This is an encoding-equivalence helper, not a controller or a JSON admission
policy. Strings/escapes and numeric spellings remain exact. Python's existing
NaN/Infinity spellings are preserved, not normalized or newly permitted.
"""
import hashlib
import json
import re

SPACE = ' \t\r\n'
PUNCTUATION = '{}[]:,'
LITERAL = re.compile(r'(?:-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?|true|false|null|NaN|-?Infinity)')


def tokens(text):
    """Yield exact token spellings; do not join adjacent scalar tokens."""
    i = 0
    while i < len(text):
        ch = text[i]
        if ch in SPACE:
            i += 1
            continue
        if ch in PUNCTUATION:
            yield ch
            i += 1
            continue
        if ch == '"':
            start = i
            i += 1
            while i < len(text):
                if text[i] == '\\':
                    i += 2
                elif text[i] == '"':
                    i += 1
                    token = text[start:i]
                    json.loads(token)  # validate escapes/control characters
                    yield token
                    break
                else:
                    i += 1
            else:
                raise ValueError('Unterminated JSON string')
            if i > len(text):
                raise ValueError('Truncated JSON escape')
            continue
        found = LITERAL.match(text, i)
        if found is None:
            raise ValueError('Unexpected JSON token at offset ' + str(i))
        end = found.end()
        if end < len(text) and text[end] not in SPACE + PUNCTUATION:
            raise ValueError('Unseparated JSON literal at offset ' + str(end))
        yield found.group()
        i = end


def signature(text):
    digest = hashlib.sha256()
    count = 0
    for token in tokens(text):
        data = token.encode('utf-8', errors='surrogatepass')
        digest.update(len(data).to_bytes(8, 'big'))
        digest.update(data)
        count += 1
    return count, digest.hexdigest()


def assert_equivalent(pretty, compact):
    if signature(pretty) != signature(compact):
        raise ValueError('JSON lexical tokens differ; change exceeds formatting')
