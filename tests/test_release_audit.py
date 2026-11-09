import json
from pathlib import Path
import subprocess

import pytest

from tools import release_audit as r


def write(root, name, content='safe'):
    path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(content)
    return path


def test_allowlist_keeps_sources_and_excludes_private_data(tmp_path):
    included=['README.md','.env.example','planner/main.py','tests/example.py',
              'plans/plan_v1.json','web/replay/data/evidence.json']
    excluded=['.env','logs/history.json','plans/real_api/qwen_response.json',
              'docs/HOST_AUDIT.json','build/a.so','planner/a.pyc','day3/file.py.bak',
              'docs/file.png:Zone.Identifier','.snapshots/old.py','.vercel/project.json']
    for name in included+excluded:write(tmp_path,name)
    assert {str(p.relative_to(tmp_path)) for p in r.candidates(tmp_path)}==set(included)


@pytest.mark.parametrize('prefix', ['sk-', 'ghp_'])
def test_secret_findings_never_echo_value(tmp_path,prefix):
    secret=prefix+'A'*40;path=write(tmp_path,'README.md','credential='+secret)
    findings=r.scan(path,tmp_path)
    assert findings and secret not in json.dumps(findings)
    assert findings[0]['line']==1


def test_nonempty_env_template_rejected(tmp_path):
    path=write(tmp_path,'.env.example','QWEN_API_KEY=not-a-real-key\n')
    assert r.scan(path,tmp_path)[0]['rule']=='NONEMPTY_TEMPLATE_CREDENTIAL'


def test_symlink_rejected(tmp_path):
    target=write(tmp_path,'private.txt');link=tmp_path/'README.md';link.symlink_to(target)
    assert r.scan(link,tmp_path)[0]['rule']=='SYMLINK_NOT_PUBLISHED'


def test_new_export_is_checksum_identical_and_no_overwrite(tmp_path):
    source=tmp_path/'source';source.mkdir();write(source,'README.md')
    output=tmp_path/'export';report=r.prepare(output,source)
    assert r.sha(output/'README.md')==report['files'][0]['sha256']
    with pytest.raises(ValueError,match='overwrite'):r.prepare(output,source)


def test_secret_blocks_export(tmp_path):
    source=tmp_path/'source';source.mkdir();write(source,'README.md','sk-'+'Z'*32)
    output=tmp_path/'export'
    with pytest.raises(ValueError):r.prepare(output,source)
    assert not output.exists()


def test_gitignore_blocks_private_artifacts_but_keeps_examples(tmp_path):
    subprocess.run(['git','init','--quiet',str(tmp_path)],check=True)
    (tmp_path/'.gitignore').write_bytes((r.ROOT/'.gitignore').read_bytes())
    ignored=['.env','.env.local','logs/task.json','build/plugin.so','.snapshots/old.json',
             '.delivery/report.json','.aws/credentials','.vercel/project.json','bundle.zip',
             'plans/real_api/qwen_response.json','docs/HOST_AUDIT.json']
    kept=['.env.example','plans/plan_v1.json','plans/plan_nl_v1.json',
          'plans/real_api/qwen_plan.json','web/replay/data/evidence.json','assets/scenes/inspection_world/v1/inspection_world.pgm']
    for name in ignored+kept:
        result=subprocess.run(['git','-C',str(tmp_path),'check-ignore','--no-index','--quiet',name])
        assert (result.returncode==0)==(name in ignored),name
