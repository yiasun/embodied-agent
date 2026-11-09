"""Allowlisted V1.0 export and redacted secret-pattern audit. Never uploads.

Reports contain paths / rule names / line numbers only, never matching values.
No Git initialization in the source tree; --prepare creates a NEW export dir.
"""
import argparse
import fnmatch
import hashlib
import json
import os
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
DIRECTORIES = ('planner','day1','day3','day4','reliability','replay','scenes','assets',
               'configs','launch','tools','tests','docs','web')
ROOT_FILES = ('.gitignore','.gitattributes','.env.example','README.md','pyproject.toml','NOTICE.md')
PLANS = ('plans/plan_v1.json','plans/plan_nl_v1.json','plans/real_api/qwen_plan.json')
FORBIDDEN_PARTS = {'__pycache__','.git','.env','.aws','.codex','.agents','.vercel',
    '.snapshots','.delivery','node_modules','.venv','venv','build','install','logs'}
FORBIDDEN_GLOBS = ('*.pyc','*.pyo','*.bak','*.tmp','*.swp','*.so','*.o','*.a',
    '*.zip','*.tar','*.tar.gz','*.tgz','*.db3','*.mcap','*.bag','*.mp4','*.mov','*:Zone.Identifier')
SECRET_RULES = {
    'PRIVATE_KEY': re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----'),
    'GITHUB_TOKEN': re.compile(rb'\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,})\b'),
    'CLOUD_API_KEY': re.compile(rb'\bsk-[A-Za-z0-9]{20,}\b'),
    'AWS_ACCESS_KEY': re.compile(rb'\b(?:AKIA|ASIA)[A-Z0-9]{16}\b'),
    'URL_PASSWORD': re.compile(rb'https?://[^\s/"\x27:]+:[^\s/"\x27@]+@'),
    'NAMED_CREDENTIAL': re.compile(rb'''(?i)["']?(?:api[_-]?key|access[_-]?token|client[_-]?secret|password|authorization)["']?\s*[:=]\s*["']([A-Za-z0-9_./+\-=]{20,})["']'''),
    'HOST_HOME_PATH': re.compile(rb'(?:/home|/Users)/[A-Za-z0-9_.-]+/'),
}
FIXTURE_WORDS = (b'synthetic',b'fixture',b'fake',b'example',b'dummy',b'test-key',b'mock')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def allowed(path):
    parts = path.parts
    return (not set(parts) & FORBIDDEN_PARTS and not any(fnmatch.fnmatch(path.name, glob) for glob in FORBIDDEN_GLOBS)
            and not path.name.startswith('.env') and not (parts[0]=='docs' and path.name.endswith('_AUDIT.json')))


def candidates(root=ROOT):
    result = []
    for name in ROOT_FILES + PLANS:
        path = root / name
        if path.is_file():result.append(path)
    for directory in DIRECTORIES:
        folder = root / directory
        if not folder.exists():continue
        for base, dirs, files in os.walk(folder, followlinks=False):
            dirs[:] = [d for d in dirs if allowed((Path(base)/d).relative_to(root)) and not (Path(base)/d).is_symlink()]
            for filename in files:
                path = Path(base)/filename
                if allowed(path.relative_to(root)):
                    result.append(path)
    return sorted(set(result), key=lambda p:str(p.relative_to(root)))


def scan(path, root=ROOT):
    name = str(path.relative_to(root)); findings=[]
    if path.is_symlink():return [{'path':name,'rule':'SYMLINK_NOT_PUBLISHED','line':None}]
    if path.stat().st_size>5*1024*1024:return [{'path':name,'rule':'OVERSIZE_REVIEW_REQUIRED','line':None}]
    raw=path.read_bytes()
    if path.name == '.env.example':
        for number, line in enumerate(raw.splitlines(), 1):
            if re.match(rb'(?i)\s*[A-Z0-9_]*(?:KEY|TOKEN|SECRET|PASSWORD)\s*=\s*\S+', line):
                findings.append({'path':name,'rule':'NONEMPTY_TEMPLATE_CREDENTIAL','line':number})
    for rule, pattern in SECRET_RULES.items():
        for match in pattern.finditer(raw):
            line=raw.count(b'\n',0,match.start())+1
            # Deliberate, named fake credentials in tests are not credentials.
            start=raw.rfind(b'\n',0,match.start())+1;end=raw.find(b'\n',match.end())
            context=raw[start:end if end>=0 else len(raw)].lower()
            if 'tests' in path.relative_to(root).parts and any(word in context for word in FIXTURE_WORDS):continue
            findings.append({'path':name,'rule':rule,'line':line})
    return findings


def audit(root=ROOT):
    files=candidates(root);findings=[item for path in files for item in scan(path,root)]
    records=[{'path':str(path.relative_to(root)),'bytes':path.stat().st_size,'sha256':sha(path)} for path in files]
    return {'audit_version':'release-allowlist-v1','file_count':len(files),'total_bytes':sum(r['bytes'] for r in records),
        'findings':findings,'files':records,
        'excluded':['raw logs','model response/provenance bundles','environment credentials','Git metadata',
                    'machine audit JSON','snapshots','builds/caches','archives and temporary files'],
        'limitations':['Pattern scanning is not proof of absence of every possible secret.',
                       'Historical raw artifacts stay local; only explicit safe plan examples and sanitized replay are included.']}


def prepare(output, root=ROOT):
    report=audit(root)
    if report['findings']:raise ValueError('Release audit failed; inspect redacted finding locations')
    if output.exists():raise ValueError('Refusing to overwrite an existing release directory')
    output.mkdir(parents=True)
    for record in report['files']:
        target=output/record['path'];target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(root/record['path'],target)
        if sha(target)!=record['sha256']:raise ValueError('Export checksum mismatch')
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare',type=Path)
    parser.add_argument('--report',type=Path,help='Private local audit output; choose .delivery/ or /tmp/')
    args=parser.parse_args(argv)
    report=prepare(args.prepare) if args.prepare else audit()
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='files'},ensure_ascii=False,indent=2))
    if report['findings']:return 1
    return 0


if __name__=='__main__':
    raise SystemExit(main())
