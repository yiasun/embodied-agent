"""One HTTP attempt only. No redirects, proxies, SDK retries or logging."""
import socket
from dataclasses import dataclass, field
from urllib.error import HTTPError, URLError
from urllib.request import Request, build_opener, HTTPRedirectHandler, ProxyHandler

from .base import PlanningError

MAX_RESPONSE_BYTES = 1024 * 1024


@dataclass(frozen=True)
class HTTPResponse:
    status: int
    body: bytes = field(repr=False)
    headers: dict = field(default_factory=dict, repr=False)


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class UrllibTransport:
    def post(self, url, *, headers, body, timeout):
        # Explicit Base URLs are the only destinations; environment proxy settings
        # and redirects must never move the Authorization header elsewhere.
        request = Request(url, data=body, headers=headers, method='POST')
        opener = build_opener(ProxyHandler({}), NoRedirect())
        try:
            with opener.open(request, timeout=timeout) as response:
                data = response.read(MAX_RESPONSE_BYTES + 1)
                if len(data) > MAX_RESPONSE_BYTES:
                    raise PlanningError('RESPONSE_TOO_LARGE')
                return HTTPResponse(response.status, data, dict(response.headers))
        except HTTPError as exc:
            try:
                data = exc.read(MAX_RESPONSE_BYTES)
                return HTTPResponse(exc.code, data, dict(exc.headers or {}))
            finally:
                exc.close()
        except (TimeoutError, socket.timeout):
            raise PlanningError('REQUEST_TIMEOUT') from None
        except URLError as exc:
            code = 'REQUEST_TIMEOUT' if isinstance(exc.reason, TimeoutError) else 'NETWORK_ERROR'
            raise PlanningError(code) from None
        except OSError:
            raise PlanningError('NETWORK_ERROR') from None
