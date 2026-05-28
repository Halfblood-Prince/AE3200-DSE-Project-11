"""Create and optionally publish the Shields JSON coverage badge."""

from __future__ import annotations

import base64
import json
import os
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


BADGE_PATH = Path(".github/badges/coverage.json")
COVERAGE_PATH = Path("coverage.json")


def badge_color(percent: float) -> str:
    """Choose a badge color based on total line coverage."""
    if percent >= 90:
        return "brightgreen"
    if percent >= 80:
        return "green"
    if percent >= 70:
        return "yellowgreen"
    if percent >= 60:
        return "yellow"
    if percent >= 50:
        return "orange"
    return "red"


def build_badge() -> dict[str, int | str]:
    """Read coverage.py JSON output and build a Shields endpoint payload."""
    with COVERAGE_PATH.open(encoding="utf-8") as coverage_file:
        coverage = json.load(coverage_file)

    percent = float(coverage["totals"]["percent_covered"])
    display = coverage["totals"]["percent_covered_display"]
    return {
        "schemaVersion": 1,
        "label": "coverage",
        "message": f"{display}%",
        "color": badge_color(percent),
    }


def write_local_badge(badge: dict[str, int | str]) -> bytes:
    """Write the local badge file and return its UTF-8 encoded contents."""
    BADGE_PATH.parent.mkdir(parents=True, exist_ok=True)
    content = json.dumps(badge, indent=2) + "\n"
    BADGE_PATH.write_text(content, encoding="utf-8")
    return content.encode("utf-8")


def write_job_summary(badge: dict[str, int | str]) -> None:
    """Append the total coverage percentage to the GitHub job summary."""
    summary_path = os.environ.get("GITHUB_STEP_SUMMARY")
    if not summary_path:
        return

    with open(summary_path, "a", encoding="utf-8") as summary:
        summary.write(f"## Coverage\n\nTotal line coverage: **{badge['message']}**\n")


def github_api_request(
    method: str,
    url: str,
    token: str,
    payload: dict[str, object] | None = None,
) -> dict[str, object]:
    """Call the GitHub REST API with a JSON request and response body."""
    data = None
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")

    request = urllib.request.Request(
        url,
        data=data,
        method=method,
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
            "X-GitHub-Api-Version": "2022-11-28",
        },
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.loads(response.read().decode("utf-8"))


def remote_badge(repo: str, branch: str, token: str) -> dict[str, object] | None:
    """Fetch the current badge file from GitHub, or return None if absent."""
    path = urllib.parse.quote(BADGE_PATH.as_posix())
    ref = urllib.parse.quote(branch)
    url = f"https://api.github.com/repos/{repo}/contents/{path}?ref={ref}"
    try:
        return github_api_request("GET", url, token)
    except urllib.error.HTTPError as error:
        if error.code == 404:
            return None
        raise


def publish_badge(content: bytes) -> None:
    """Update the badge file on the current GitHub branch without local git."""
    token = os.environ["GITHUB_TOKEN"]
    repo = os.environ["GITHUB_REPOSITORY"]
    branch = os.environ["GITHUB_REF_NAME"]
    path = urllib.parse.quote(BADGE_PATH.as_posix())
    url = f"https://api.github.com/repos/{repo}/contents/{path}"
    current = remote_badge(repo, branch, token)

    payload: dict[str, object] = {
        "message": "Update coverage badge",
        "content": base64.b64encode(content).decode("ascii"),
        "branch": branch,
    }

    if current is not None:
        existing = base64.b64decode(str(current["content"]))
        if existing == content:
            print("Coverage badge is already up to date.")
            return
        payload["sha"] = current["sha"]

    github_api_request("PUT", url, token, payload)
    print("Coverage badge updated.")


def main() -> None:
    """Update the local badge and publish it when requested by CI."""
    badge = build_badge()
    content = write_local_badge(badge)
    write_job_summary(badge)
    if os.environ.get("UPDATE_REMOTE_BADGE", "").lower() in {"1", "true", "yes"}:
        publish_badge(content)


if __name__ == "__main__":
    main()
