# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from waflib import Errors

# Each set of boards and their capabilities.
# To use these, import as follows:
# ```
# from platform_capabilities import get_capability_dict
# ```

# Master key set. Any capabilities that are added to any platform have to be added here.
# Once added, add the capability to ALL capability dictionaries with the appropriate value.

JAVASCRIPT_BYTECODE_VERSION = 1

master_capability_set = {
    'COMPOSITOR_USES_DMA',
    'HAS_ACCESSORY_CONNECTOR',
    'HAS_ALS_OPT3001',
    'HAS_APPLE_MFI',
    'HAS_APP_GLANCES',
    'HAS_BUILTIN_HRM',
    'HAS_CORE_NAVIGATION4',
    'HAS_DEFECTIVE_FW_CRC',
    'HAS_GLYPH_BITMAP_CACHING',
    'HAS_HARDWARE_PANIC_SCREEN',
    'HAS_HEALTH_TRACKING',
    'HAS_JAVASCRIPT',
    'HAS_LAUNCHER4',
    'HAS_LED',
    'HAS_MAGNETOMETER',
    'HAS_MAPPABLE_FLASH',
    'HAS_MASKING',
    'HAS_MICROPHONE',
    'HAS_PMIC',
    'HAS_SDK_SHELL4',
    'HAS_SPRF_V3',
    'HAS_TEMPERATURE',
    'HAS_TIMELINE_PEEK',
    'HAS_TOUCHSCREEN',
    'HAS_VIBE_SCORES',
    'HAS_VIBE_DRV2604',
    'HAS_WEATHER',
    'USE_PARALLEL_FLASH',
    'HAS_PUTBYTES_PREACKING',
    'HAS_FLASH_OTP',
}

board_capability_dicts = [
    {
        'boards': ['bb2', 'ev2_4', 'v1_5'],
        'capabilities':
        {
            'HAS_APPLE_MFI',
            'HAS_DEFECTIVE_FW_CRC',
            'HAS_MAGNETOMETER',
        },
    },
    {
        'boards': ['v2_0'],
        'capabilities':
        {
            'HAS_APPLE_MFI',
            'HAS_DEFECTIVE_FW_CRC',
            'HAS_LED',
            'HAS_MAGNETOMETER',
        },
    },
    {
        'boards': ['snowy_evt2'],
        'capabilities':
        {
            'COMPOSITOR_USES_DMA',
            'HAS_ACCESSORY_CONNECTOR',
            'HAS_APPLE_MFI',
            'HAS_APP_GLANCES',
            'HAS_CORE_NAVIGATION4',
            'HAS_DEFECTIVE_FW_CRC',
            'HAS_HEALTH_TRACKING',
            'HAS_JAVASCRIPT',
            'HAS_LAUNCHER4',
            'HAS_MAGNETOMETER',
            'HAS_MAPPABLE_FLASH',
            'HAS_MASKING',
            # 'HAS_MICROPHONE', -- TODO: disabled because driver was removed
            'HAS_PMIC',
            'HAS_SDK_SHELL4',
            'HAS_TEMPERATURE',
            'HAS_TIMELINE_PEEK',
            'HAS_VIBE_SCORES',
            'USE_PARALLEL_FLASH',
            'HAS_WEATHER',
        },
    },
    {
        'boards': ['snowy_bb2', 'snowy_dvt', 'snowy_s3'],
        'capabilities':
        {
            'COMPOSITOR_USES_DMA',
            'HAS_ACCESSORY_CONNECTOR',
            'HAS_APPLE_MFI',
            'HAS_APP_GLANCES',
            'HAS_CORE_NAVIGATION4',
            'HAS_DEFECTIVE_FW_CRC',
            'HAS_HARDWARE_PANIC_SCREEN',
            'HAS_HEALTH_TRACKING',
            'HAS_JAVASCRIPT',
            'HAS_LAUNCHER4',
            'HAS_MAGNETOMETER',
            'HAS_MAPPABLE_FLASH',
            'HAS_MASKING',
            # 'HAS_MICROPHONE', -- TODO: disabled because driver was removed
            'HAS_PMIC',
            'HAS_SDK_SHELL4',
            'HAS_TEMPERATURE',
            'HAS_TIMELINE_PEEK',
            'HAS_VIBE_SCORES',
            'USE_PARALLEL_FLASH',
            'HAS_WEATHER',
        },
    },
    {
        'boards': ['spalding_bb2'],
        'capabilities':
        {
            'COMPOSITOR_USES_DMA',
            'HAS_ACCESSORY_CONNECTOR',
            'HAS_APP_GLANCES',
            'HAS_CORE_NAVIGATION4',
            'HAS_DEFECTIVE_FW_CRC',
            'HAS_HARDWARE_PANIC_SCREEN',
            'HAS_HEALTH_TRACKING',
            'HAS_JAVASCRIPT',
            'HAS_LAUNCHER4',
            'HAS_MAGNETOMETER',
            'HAS_MAPPABLE_FLASH',
            'HAS_MASKING',
            # 'HAS_MICROPHONE', -- TODO: disabled because driver was removed
            'HAS_PMIC',
            'HAS_SDK_SHELL4',
            'HAS_TEMPERATURE',
            'HAS_VIBE_SCORES',
            'USE_PARALLEL_FLASH',
            'HAS_WEATHER',
        },
    },
    {
        'boards': ['spalding_evt', 'spalding'],
        'capabilities':
        {
            'COMPOSITOR_USES_DMA',
            'HAS_ACCESSORY_CONNECTOR',
            'HAS_APP_GLANCES',
            'HAS_CORE_NAVIGATION4',
            'HAS_DEFECTIVE_FW_CRC',
            'HAS_HARDWARE_PANIC_SCREEN',
            'HAS_HEALTH_TRACKING',
            'HAS_JAVASCRIPT',
            'HAS_LAUNCHER4',
            'HAS_MAGNETOMETER',
            'HAS_MAPPABLE_FLASH',
            'HAS_MASKING',
            # 'HAS_MICROPHONE', -- TODO: disabled because driver was removed
            'HAS_PMIC',
            'HAS_SDK_SHELL4',
            'HAS_TEMPERATURE',
            'HAS_VIBE_SCORES',
            'USE_PARALLEL_FLASH',
            'HAS_WEATHER',
        },
    },
    {
        'boards': ['silk_bb', 'silk_evt', 'silk_bb2', 'silk'],
        'capabilities':
        {
            'HAS_ACCESSORY_CONNECTOR',
            'HAS_APP_GLANCES',
            'HAS_BUILTIN_HRM',
            'HAS_CORE_NAVIGATION4',
            'HAS_HEALTH_TRACKING',
            'HAS_JAVASCRIPT',
            'HAS_LAUNCHER4',
            # 'HAS_MAPPABLE_FLASH' -- TODO: PBL-33860 verify memory-mappable flash works on silk before activating
            # 'HAS_MICROPHONE', -- TODO: disabled because driver was removed
            # 'USE_PARALLEL_FLASH' -- FIXME hack to get the "modern" flash layout. Fix when we add support for new flash
            'HAS_PMIC',
            'HAS_SDK_SHELL4',
            'HAS_SPRF_V3',
            'HAS_TEMPERATURE',
            'HAS_TIMELINE_PEEK',
            'HAS_VIBE_SCORES',
            'HAS_WEATHER',
            'HAS_PUTBYTES_PREACKING'
        },
    },
    {
        'boards': ['robert_bb', 'robert_bb2', 'robert_evt'],
        'capabilities':
        {
            'COMPOSITOR_USES_DMA',
            'HAS_ACCESSORY_CONNECTOR',
            'HAS_APP_GLANCES',
            'HAS_BUILTIN_HRM',
            'HAS_CORE_NAVIGATION4',
            'HAS_GLYPH_BITMAP_CACHING',
            'HAS_HEALTH_TRACKING',
            'HAS_JAVASCRIPT',
            'HAS_LAUNCHER4',
            'HAS_MAGNETOMETER',
            'HAS_MASKING',
            # 'HAS_MICROPHONE', -- TODO: disabled because driver was removed
            'HAS_PMIC',
            'HAS_SDK_SHELL4',
            'HAS_SPRF_V3',
            'HAS_TEMPERATURE',
            'HAS_TIMELINE_PEEK',
            'HAS_VIBE_SCORES',
            'HAS_WEATHER',
            'HAS_PUTBYTES_PREACKING'
        }
    },
    {
        'boards': ['cutts_bb'],
        'capabilities':
        {
            'COMPOSITOR_USES_DMA',
            'HAS_ACCESSORY_CONNECTOR',
            'HAS_APP_GLANCES',
            'HAS_CORE_NAVIGATION4',
            'HAS_GLYPH_BITMAP_CACHING',
            'HAS_HEALTH_TRACKING',
            'HAS_JAVASCRIPT',
            'HAS_LAUNCHER4',
            'HAS_MAGNETOMETER',
            'HAS_MASKING',
            'HAS_MICROPHONE',
            'HAS_PMIC',
            'HAS_SDK_SHELL4',
            'HAS_SPRF_V3',
            'HAS_TEMPERATURE',
            'HAS_TIMELINE_PEEK',
            'HAS_VIBE_SCORES',
            'HAS_WEATHER',
            'HAS_PUTBYTES_PREACKING',
            'HAS_TOUCHSCREEN'
        }
    },
    {
        'boards': [ 'asterix' ],
        'capabilities':
        {
            'HAS_ALS_OPT3001',
            'HAS_APP_GLANCES',
            'HAS_CORE_NAVIGATION4',
            'HAS_HEALTH_TRACKING',
            'HAS_JAVASCRIPT',
            'HAS_LAUNCHER4',
            # 'HAS_MAPPABLE_FLASH' -- TODO: PBL-33860 verify memory-mappable flash works on silk before activating
            # 'HAS_MICROPHONE',
            # 'USE_PARALLEL_FLASH' -- FIXME hack to get the "modern" flash layout. Fix when we add support for new flash
            'HAS_SDK_SHELL4',
            'HAS_SPRF_V3',
            'HAS_TEMPERATURE',
            'HAS_TIMELINE_PEEK',
            'HAS_VIBE_SCORES',
            'HAS_WEATHER',
            'HAS_PUTBYTES_PREACKING',
            # 'HAS_MAGNETOMETER',
            'HAS_VIBE_DRV2604',
            'HAS_PMIC',
            'HAS_FLASH_OTP',
        },
    },
]

# Run through again and make sure all sets include only valid keys defined in
# `master_capability_set`
boards_seen = set()

for board_dict in board_capability_dicts:
    capabilities_of_board = board_dict['capabilities']
    boards = board_dict['boards']

    # Check for duplicate boards using the intersection of boards already seen and the boards
    # in the dict we are operating on. After the check, add the ones seen to the set
    duped_boards = boards_seen.intersection(boards)
    if duped_boards:
        raise ValueError('There are multiple capability sets for the boards {!r}'
                         .format(duped_boards))
    boards_seen.update(boards)

    # Check for capabilities that aren't in the master_capability_set
    unknown_capabilities = capabilities_of_board - master_capability_set
    if unknown_capabilities:
        raise ValueError('The capability set for boards {!r} contains unknown '
                         'capabilities {!r}'.format(boards, unknown_capabilities))


def get_capability_dict(ctx, board):
    capabilities_of_board = None
    # Find capability set for board
    for capability_dict in board_capability_dicts:
        if board in capability_dict['boards']:
            capabilities_of_board = capability_dict['capabilities']

    if not capabilities_of_board:
        raise KeyError('Capability set for board: "{}" is missing or undefined'.format(board))

    # Overrides
    # If you want the capabilities to change depending on the configure/build environment, add
    # them here.

    if ctx.env.QEMU:
        # Disable smartstraps on QEMU builds
        capabilities_of_board.discard('HAS_ACCESSORY_CONNECTOR')

    if ctx.env.NOJS:
        capabilities_of_board.discard('HAS_JAVASCRIPT')

    # End overrides section

    false_capabilities = master_capability_set - capabilities_of_board
    cp_dict = {key: True for key in capabilities_of_board}
    cp_dict.update({key: False for key in false_capabilities})

    # inject expected JS bytecode version
    if cp_dict.get('HAS_JAVASCRIPT', False):
        cp_dict['JAVASCRIPT_BYTECODE_VERSION'] = JAVASCRIPT_BYTECODE_VERSION

    return cp_dict
