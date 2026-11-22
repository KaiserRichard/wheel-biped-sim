#!/usr/bin/env python3
"""Small stdlib-only formatter rule checker for the docs/lessons study pack."""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = [ROOT / 'docs/lessons/README.md', *sorted((ROOT / 'docs/lessons').glob('0[1-6]*.md'))]

def lint(path):
    issues=[]; lines=path.read_text(encoding='utf-8').splitlines(); rel=path.relative_to(ROOT)
    def add(n,msg): issues.append(f'{rel}:{n}: {msg}')
    fence=None; display=False; heading=[]; summary=None; table_cols=None; mermaid=[]
    for i,line in enumerate(lines,1):
        stripped=line.strip()
        m=re.match(r'^\s*(```+|~~~+)(.*)$',line)
        if m:
            mark,tag=m.groups()
            if fence is None:
                if line[:len(line)-len(line.lstrip())]: add(i,'code fence is indented')
                if not tag.strip(): add(i,'code fence needs a language tag')
                fence=(mark[0],len(mark)); mermaid=[]
            elif mark[0]==fence[0] and len(mark)>=fence[1]:
                if fence[0]=='`' and 'mermaid' in (line.lower()):
                    pass
                fence=None
            continue
        if fence:
            if 'mermaid' in lines[max(0,i-2)].lower(): mermaid.append(stripped)
            continue
        if stripped.startswith('<summary>'):
            summary=i
            if i < len(lines) and lines[i].strip(): add(i+1,'blank line required after summary')
        if stripped=='</details>' and i>1 and lines[i-2].strip(): add(i,'blank line required before </details>')
        if stripped=='$$':
            if not display:
                if i>1 and lines[i-2].strip(): add(i,'blank line required before display math')
                display=True
            else:
                if i < len(lines) and lines[i].strip(): add(i,'blank line required after display math')
                display=False
        elif '$$' in line:
            add(i,'display math delimiters must be alone on their lines')
        # Inline math delimiters; display regions excluded.
        if not display and '$' in line and not stripped.startswith('$$'):
            for mt in re.finditer(r'(?<!\\)\$(.+?)(?<!\\)\$',line):
                expr=mt.group(1)
                if expr[:1].isspace() or expr[-1:].isspace(): add(i,'inline math must not start or end with a space')
                if '|' in expr: add(i,'escape | inside inline math or move math out of table')
        hm=re.match(r'^(#{1,6})\s+',line)
        if hm:
            level=len(hm.group(1)); heading.append((i,level))
            if i>1 and level>heading[-2][1]+1: add(i,'heading level skipped')
        # GFM table shape, ignoring escaped pipes and code spans.
        if '|' in line and not line.lstrip().startswith('```'):
            cells=[c.strip() for c in re.sub(r'`[^`]*`','',line).strip().strip('|').split('|')]
            if table_cols is None:
                if i<len(lines) and re.match(r'^\s*\|?\s*:?-{3,}',lines[i]):
                    table_cols=len(cells)
                    sep=[c.strip() for c in lines[i].strip().strip('|').split('|')]
                    if len(sep)!=table_cols: add(i+1,'table separator column count differs from header')
                else: table_cols=None
            else:
                if not stripped:
                    table_cols=None
                elif len(cells)!=table_cols: add(i,f'table row has {len(cells)} columns; expected {table_cols}')
                if '`' in line and any('|' in span for span in re.findall(r'`([^`]*)`', line)): add(i,'pipe inside table code must be escaped')
        elif not stripped: table_cols=None
    if fence: add(len(lines),'unclosed code fence')
    if display: add(len(lines),'unclosed display math block')
    if not heading or heading[0][1]!=1: add(1,'file must begin with one H1')
    if sum(level==1 for _,level in heading)!=1: add(1,'file must contain exactly one H1')
    return issues

files=[Path(a) if Path(a).is_absolute() else ROOT/ a for a in sys.argv[1:]] or DEFAULT
all_issues=[]
for f in files: all_issues.extend(lint(f))
if all_issues:
    print('\n'.join(all_issues)); print(f'{len(all_issues)} issue(s)'); sys.exit(1)
print(f'0 issues across {len(files)} file(s)')
