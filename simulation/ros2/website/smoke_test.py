from __future__ import annotations

import argparse
import http.cookiejar
import urllib.error
import urllib.parse
import urllib.request


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def request(opener, url, data=None, follow_redirects=True):
    if data is not None:
        data = urllib.parse.urlencode(data).encode()

    try:
        response = opener.open(url, data=data, timeout=10)
        body = response.read().decode("utf-8", errors="replace")
        return response.status, response.headers, body
    except urllib.error.HTTPError as error:
        body = error.read().decode("utf-8", errors="replace")
        return error.code, error.headers, body


def assert_status(actual, expected, label):
    if actual != expected:
        raise AssertionError(f"{label}: expected HTTP {expected}, got {actual}")


def assert_contains(value, expected, label):
    if expected not in value:
        raise AssertionError(f"{label}: expected to contain {expected!r}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:8080")
    args = parser.parse_args()

    cookie_jar = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(
        urllib.request.HTTPCookieProcessor(cookie_jar), NoRedirect
    )
    redirecting_opener = urllib.request.build_opener(
        urllib.request.HTTPCookieProcessor(cookie_jar)
    )

    status, _, body = request(opener, f"{args.base_url}/login")
    assert_status(status, 200, "login page")
    assert_contains(body, "Operator Sign In", "login page")

    status, headers, _ = request(opener, f"{args.base_url}/mission/alpha-0426")
    assert_status(status, 302, "protected dashboard redirect")
    assert_contains(headers.get("Location", ""), "/login", "protected redirect")

    status, _, body = request(opener, f"{args.base_url}/api/mission/alpha-0426")
    assert_status(status, 401, "protected API rejection")
    assert_contains(body, "authentication_required", "protected API rejection")

    status, headers, _ = request(
        opener,
        f"{args.base_url}/login",
        {"username": "admin", "password": "admin"},
    )
    assert_status(status, 303, "login redirect")
    assert_contains(headers.get("Location", ""), "/mission/alpha-0426", "login redirect")

    status, _, body = request(redirecting_opener, f"{args.base_url}/mission/alpha-0426")
    assert_status(status, 200, "authenticated dashboard")
    assert_contains(body, "Mission: ALPHA-0426", "authenticated dashboard")

    status, _, body = request(opener, f"{args.base_url}/api/mission/alpha-0426")
    assert_status(status, 200, "authenticated API")
    assert_contains(body, "Sentinel-7B", "authenticated API")

    print("Flask AeroSentinel smoke tests passed.")


if __name__ == "__main__":
    main()
