#!/usr/bin/env python3
"""Generate STATUS.md (and the summary block in README.md) from GitHub issues.

Run by .github/workflows/status.yml. Can also be run locally:
    GITHUB_TOKEN=... GITHUB_REPOSITORY=owner/repo python tools/update_status.py
Standard library only.
"""
import json
import os
import re
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REPO = os.environ.get("GITHUB_REPOSITORY", "")
TOKEN = os.environ.get("GITHUB_TOKEN", "")
API = f"https://api.github.com/repos/{REPO}"

WORKSTREAMS = {
    "A": "Drone & infrastructure",
    "B": "Calibration & marker map",
    "C": "Localization & TF",
    "D": "Simulation",
    "E": "State estimation (EKF)",
    "F": "Control",
    "G": "Perception (balloon)",
    "H": "Mission & interception",
    "I": "Evaluation & deliverables",
}


def get(url):
    items, page = [], 1
    while True:
        sep = "&" if "?" in url else "?"
        req = urllib.request.Request(f"{url}{sep}per_page=100&page={page}")
        req.add_header("Accept", "application/vnd.github+json")
        if TOKEN:
            req.add_header("Authorization", f"Bearer {TOKEN}")
        with urllib.request.urlopen(req) as resp:
            batch = json.load(resp)
        items.extend(batch)
        if len(batch) < 100:
            return items
        page += 1


def bar(done, total, width=20):
    if total == 0:
        return "`" + "░" * width + "`"
    filled = round(width * done / total)
    return "`" + "█" * filled + "░" * (width - filled) + "`"


def pct(done, total):
    return f"{round(100 * done / total)}%" if total else "–"


def task_id(title):
    m = re.match(r"\[([A-Z]\d+)\]", title)
    return m.group(1) if m else ""


def sort_key(t):
    tid = t["id"] or "Z99"
    return (tid[0], int(tid[1:]) if tid[1:].isdigit() else 99)


def main():
    if not REPO:
        sys.exit("GITHUB_REPOSITORY is not set")

    milestones = {m["number"]: m for m in get(f"{API}/milestones?state=all")}
    raw = [i for i in get(f"{API}/issues?state=all") if "pull_request" not in i]

    tasks = []
    for i in raw:
        labels = {l["name"] for l in i["labels"]}
        ws = next((l.split(":", 1)[1] for l in labels if l.startswith("ws:")), "?")
        deps_line = next((ln for ln in (i.get("body") or "").splitlines()
                          if ln.startswith("**Depends on:**")), "")
        tasks.append({
            "number": i["number"],
            "id": task_id(i["title"]),
            "title": re.sub(r"^\[[A-Z]\d+\]\s*", "", i["title"]),
            "url": i["html_url"],
            "closed": i["state"] == "closed",
            "closed_at": i.get("closed_at") or "",
            "assignees": [a["login"] for a in i["assignees"]],
            "ws": ws,
            "ms": (i.get("milestone") or {}).get("number"),
            "critical": "critical-path" in labels,
            "stretch": "stretch" in labels,
            "blocked": "blocked" in labels,
            "deps": [int(n) for n in re.findall(r"#(\d+)", deps_line)],
        })
    by_num = {t["number"]: t for t in tasks}

    def ready(t):
        return all(by_num.get(d, {"closed": True})["closed"] for d in t["deps"])

    def link(t):
        tag = f"**{t['id']}**" if t["id"] else f"#{t['number']}"
        flags = (" 🔴" if t["critical"] else "") + (" ✨" if t["stretch"] else "")
        return f"[{tag}]({t['url']}) {t['title']}{flags}"

    def ms_name(t):
        m = milestones.get(t["ms"])
        return m["title"].split(":")[0] if m else "–"

    def ms_due(num):
        m = milestones.get(num)
        return (m.get("due_on") or "9999")[:10] if m else "9999"

    total = len(tasks)
    done = sum(t["closed"] for t in tasks)
    in_prog = [t for t in tasks if not t["closed"] and t["assignees"] and not t["blocked"]]
    blocked = [t for t in tasks if not t["closed"] and t["blocked"]]
    free = [t for t in tasks if not t["closed"] and not t["assignees"] and not t["blocked"]]
    free_ready = sorted([t for t in free if ready(t)],
                        key=lambda t: (t["stretch"], ms_due(t["ms"]), not t["critical"], sort_key(t)))
    free_waiting = sorted([t for t in free if not ready(t)],
                          key=lambda t: (ms_due(t["ms"]), sort_key(t)))

    # Milestone table
    ms_rows = []
    for num in sorted(milestones, key=ms_due):
        m = milestones[num]
        ts = [t for t in tasks if t["ms"] == num]
        d = sum(t["closed"] for t in ts)
        ms_rows.append(f"| [{m['title']}]({m['html_url']}) | {ms_due(num)} | {bar(d, len(ts))} {pct(d, len(ts))} | {d}/{len(ts)} |")

    # Workstream table
    ws_rows = []
    for key, name in WORKSTREAMS.items():
        ts = [t for t in tasks if t["ws"] == key]
        if not ts:
            continue
        d = sum(t["closed"] for t in ts)
        ip = sum(1 for t in ts if t in in_prog)
        ws_rows.append(f"| **{key}** · {name} | {bar(d, len(ts), 12)} {pct(d, len(ts))} | {d}/{len(ts)} | {ip} |")

    # People
    people = {}
    for t in in_prog:
        for a in t["assignees"]:
            people.setdefault(a, []).append(t)
    done_by = {}
    for t in tasks:
        if t["closed"]:
            for a in t["assignees"]:
                done_by[a] = done_by.get(a, 0) + 1

    out = [
        "# Project Status",
        "",
        "> 🤖 Auto-generated from the GitHub issues by `tools/update_status.py` on every issue change. **Do not edit by hand**: update the issues instead.",
        "> 🔴 = critical path · ✨ = stretch",
        "",
        f"## Overall: {done} / {total} tasks done ({pct(done, total)})",
        "",
        bar(done, total, 40),
        "",
        "```mermaid",
        "pie showData title Tasks",
        f'    "Done" : {done}',
        f'    "In progress" : {len(in_prog)}',
        f'    "Up for grabs" : {len(free_ready)}',
        f'    "Waiting on dependencies" : {len(free_waiting)}',
        f'    "Blocked" : {len(blocked)}',
        "```",
        "",
        "## Milestones",
        "",
        "| Milestone | Due | Progress | Done |",
        "|---|---|---|---|",
        *ms_rows,
        "",
        "## Workstreams",
        "",
        "| Workstream | Progress | Done | In progress |",
        "|---|---|---|---|",
        *ws_rows,
        "",
        "## Who is doing what",
        "",
    ]
    if people:
        out += ["| Person | Working on | Done so far |", "|---|---|---|"]
        for p in sorted(people):
            items = "<br>".join(link(t) for t in sorted(people[p], key=sort_key))
            out.append(f"| @{p} | {items} | {done_by.get(p, 0)} |")
    else:
        out.append("_Nobody has claimed a task yet. Assign yourself on an issue!_")

    out += ["", "## 🙋 Up for grabs: ready to start now", "",
            "Dependencies are done and nobody is assigned. **Pick from the top.**", ""]
    if free_ready:
        out += ["| Task | Workstream | Milestone |", "|---|---|---|"]
        out += [f"| {link(t)} | {t['ws']} | {ms_name(t)} |" for t in free_ready]
    else:
        out.append("_Nothing free right now._")

    if blocked:
        out += ["", "## ⛔ Blocked", "", "| Task | Owner |", "|---|---|"]
        out += [f"| {link(t)} | {', '.join('@' + a for a in t['assignees']) or '–'} |" for t in blocked]

    out += ["", "<details><summary>⏳ Waiting on dependencies (" + str(len(free_waiting)) + ")</summary>", "",
            "| Task | Waiting on | Milestone |", "|---|---|---|"]
    for t in free_waiting:
        waits = ", ".join(by_num[d]["id"] or f"#{d}" for d in t["deps"] if d in by_num and not by_num[d]["closed"])
        out.append(f"| {link(t)} | {waits} | {ms_name(t)} |")
    out += ["", "</details>", ""]

    recent = sorted([t for t in tasks if t["closed"]], key=lambda t: t["closed_at"], reverse=True)[:10]
    out += ["## ✅ Recently done", ""]
    if recent:
        out += [f"- {t['closed_at'][:10]}: {link(t)}" for t in recent]
    else:
        out.append("_Nothing yet: let's go!_")
    out.append("")

    (ROOT / "STATUS.md").write_text("\n".join(out), encoding="utf-8")

    # README summary block
    readme = ROOT / "README.md"
    text = readme.read_text(encoding="utf-8")
    summary = "\n".join([
        "<!-- STATUS:START -->",
        f"### 📊 Progress: {done}/{total} tasks ({pct(done, total)}) · {len(in_prog)} in progress · {len(free_ready)} up for grabs",
        "",
        bar(done, total, 40),
        "",
        "| Milestone | Due | Progress |",
        "|---|---|---|",
        *["| " + " | ".join(r.strip("|").split("|")[:3]).strip() + " |" for r in ms_rows],
        "",
        "➡️ **Details, owners and free tasks: [STATUS.md](STATUS.md)**",
        "<!-- STATUS:END -->",
    ])
    text = re.sub(r"<!-- STATUS:START -->.*?<!-- STATUS:END -->", lambda _: summary, text, flags=re.S)
    readme.write_text(text, encoding="utf-8")
    print(f"STATUS.md updated: {done}/{total} done")


if __name__ == "__main__":
    main()
