import copy
import json

import pytest

from replay.public import ROOT, load_public, verify_public


@pytest.fixture(scope='module')
def data():
    return load_public()


def test_public_metrics_recomputed(data):
    result=verify_public(data)
    assert result['D3']['minimum_sampled_clearance_m']==pytest.approx(.3529465546)
    assert result['D3']['observed_y_reversals']==3


@pytest.mark.parametrize('change',['speed','position','clearance','projection','order','connect','plan','assets','event'])
def test_public_tampering_rejected(data,change):
    altered=copy.deepcopy(data);d3=altered['experiments']['D3']
    if change=='speed':d3['frames'][1]['speed_m_s']=0
    if change=='position':d3['frames'][1]['robot'][0]+=1
    if change=='clearance':d3['frames'][-1]['clearance_m']=1
    if change=='projection':d3['frames'][1]['projection'][0]+=1
    if change=='order':d3['frames'][1]['t']=-1
    if change=='connect':d3['frames'][1]['source_row']+=100
    if change=='plan':d3['binding']['plan_id']='0'*64
    if change=='assets':altered['scene_assets']['revision']='9.0.0'
    if change=='event':d3['events'][-1]['robot']=[0,0,0]
    with pytest.raises(ValueError):verify_public(altered)


def test_embedded_browser_data_matches_download():
    text=(ROOT/'web/replay/evidence.js').read_text()
    assert text.startswith('window.REPLAY_DATA=') and text.endswith(';\n')
    assert json.loads(text[len('window.REPLAY_DATA='):-2])==load_public()
