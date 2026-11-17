"""Map file I/O and the map_diff command (003-R33, R34). Files live in pytest's tmp_path."""
import numpy as np
import pytest
import yaml

from grid_fixtures import boxed_room
from vacuum_mapping.map_diff_cli import main
from vacuum_mapping.map_io import read_map, write_map


def test_003_R33_write_then_read_roundtrips_cells_resolution_and_origin(tmp_path):
    raster = boxed_room(origin=(-1.5, 2.0))
    raster.cells[5, 7] = -1  # an unknown cell inside the room
    yaml_path, pgm_path = write_map(raster, str(tmp_path / 'm.yaml'))
    back = read_map(yaml_path)
    assert np.array_equal(back.cells, raster.cells)
    assert (back.resolution, back.origin_x, back.origin_y) == (0.05, -1.5, 2.0)
    assert pgm_path.endswith('m.pgm')


def test_003_R33_pgm_rows_are_flipped_to_ros_layout(tmp_path):
    raster = boxed_room(rows=6, cols=8)
    raster.cells[1, 1:7] = 100  # wall near the LOW y side
    write_map(raster, str(tmp_path / 'm.yaml'))
    pixels = (tmp_path / 'm.pgm').read_bytes()[-48:]
    # the last image row (bottom) must be the first raster row: all occupied (black)
    assert set(pixels[-8:]) == {0}


def test_003_R33_pgm_comment_lines_are_accepted(tmp_path):
    (tmp_path / 'c.pgm').write_bytes(b'P5\n# made by map_saver\n2 1\n255\n\x00\xfe')
    (tmp_path / 'c.yaml').write_text('image: c.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
                                     'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    cells = read_map(str(tmp_path / 'c.yaml')).cells
    assert cells.tolist() == [[100, 0]]


def test_003_R33_nonzero_yaw_is_rejected(tmp_path):
    yaml_path, _ = write_map(boxed_room(), str(tmp_path / 'm.yaml'))
    meta = yaml.safe_load(open(yaml_path))
    meta['origin'][2] = 0.5
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(meta, f)
    with pytest.raises(ValueError):
        read_map(yaml_path)


def test_003_R34_cli_prints_report_writes_file_and_leaves_maps_untouched(tmp_path, capsys):
    a, b = boxed_room(), boxed_room()
    b.cells[10:20, 30] = 100
    write_map(a, str(tmp_path / 'a.yaml'))
    write_map(b, str(tmp_path / 'b.yaml'))
    before = (tmp_path / 'a.pgm').read_bytes(), (tmp_path / 'b.pgm').read_bytes()
    out = tmp_path / 'report.md'
    code = main([str(tmp_path / 'a.yaml'), str(tmp_path / 'b.yaml'), '-o', str(out)])
    assert code == 0
    assert 'Pixel difference' in capsys.readouterr().out
    assert 'Pixel difference' in out.read_text()
    assert before == ((tmp_path / 'a.pgm').read_bytes(), (tmp_path / 'b.pgm').read_bytes())


def test_003_R34_cli_reports_missing_file_with_exit_code_2(tmp_path, capsys):
    assert main([str(tmp_path / 'no.yaml'), str(tmp_path / 'no2.yaml')]) == 2
    assert 'map_diff' in capsys.readouterr().err
